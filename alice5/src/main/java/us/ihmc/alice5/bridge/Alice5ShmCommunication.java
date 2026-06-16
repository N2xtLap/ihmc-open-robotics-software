package us.ihmc.alice5.bridge;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.alice5.Alice5SensorInformation;
import us.ihmc.avatar.wholeBodyHardwareControl.HardwareCommunicationInterface;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Hardware communication over the ALICE5 WBC shared-memory bridge.
 * <p>
 * read(): seqlock state snapshot to SensorDataContext (joints, pelvis IMU, foot F/T).
 * write(): JointDesiredOutputList to the command block, with the safety logic of
 * bridge_spec.md rev.2: INIT echo handshake, freeze to DAMPING_REQUEST, re-engage re-home.
 */
public class Alice5ShmCommunication implements HardwareCommunicationInterface
{
   private static final String LEFT_FOOT_FT_SENSOR = "LeftFootFTSensor";
   private static final String RIGHT_FOOT_FT_SENSOR = "RightFootFTSensor";

   private static final long FREEZE_THRESHOLD_NS = 20_000_000L; // 20 ms
   private static final long FAULT_NAN_BIT = 1L; // robot-side latched NaN-command fault (blocks re-engage)
   private static final long FIRST_STATE_OBSERVATION_NS = 500_000_000L; // 0.5 s
   private static final int ALL_JOINTS_ENABLE_MASK = 0x7FFFFF;
   // foot_ft_validity bits (ABI v2): bit0/1 = left/right wrench valid, bit2/3 = left/right CoP valid.
   private static final int[] WRENCH_VALID_BIT = {0x1, 0x2};

   private final Alice5ShmBridge bridge;
   private final String[] orderedJointNames;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean robotFaulted = new YoBoolean("shmRobotFaulted", registry);
   private final YoBoolean stateFrozen = new YoBoolean("shmStateFrozen", registry);
   private final YoBoolean receivedFirstState = new YoBoolean("shmReceivedFirstState", registry);
   private final YoInteger safetyState = new YoInteger("shmSafetyState", registry);
   private final YoLong staleReadExhaustedCount = new YoLong("shmStaleReadExhaustedCount", registry);
   private final YoLong stateSequenceNumber = new YoLong("shmStateSequenceNumber", registry);
   private final YoLong commandHeartbeat = new YoLong("shmCommandHeartbeat", registry);

   private final Alice5ShmBridge.StateSnapshot scratchSnapshot = new Alice5ShmBridge.StateSnapshot();
   private final Alice5ShmBridge.StateSnapshot latestSnapshot = new Alice5ShmBridge.StateSnapshot();
   private boolean hasSnapshot = false;

   // Sensor data context targets, cached at registration.
   private boolean contextRegistered = false;
   private final LowLevelState[] jointStates;
   private ImuData imuData;
   private final DMatrixRMaj[] footWrenches = new DMatrixRMaj[2];
   // SIM-EXT FT: last valid sole-frame wrench per foot (IHMC order: Tx Ty Tz Fx Fy Fz) for stale-hold
   // when the loadcell wrench validity bit drops, plus an alarm raised while a foot is held stale.
   private final double[][] lastValidWrench = {new double[6], new double[6]};
   private final boolean[] hasValidWrench = new boolean[2];
   private final YoBoolean footWrenchStale = new YoBoolean("shmFootWrenchStale", registry);

   // Staleness / first state tracking (all in System.nanoTime base).
   private long lastSeqChangeNs = 0;
   private long lastReadCallNs = 0;
   private long firstSnapshotNs = 0;
   private long firstObservedSeq = -1;
   private long firstObservedCycle = -1;
   private volatile boolean hasFirstStateVolatile = false;

   // Write side.
   private final Quaternion tempOrientation = new Quaternion();
   private final Vector3D tempVector = new Vector3D();
   private final double[] qDesired = new double[Alice5ShmBridge.N_JOINTS];
   private final double[] qdDesired = new double[Alice5ShmBridge.N_JOINTS];
   private final double[] tauFeedForward = new double[Alice5ShmBridge.N_JOINTS];
   private final double[] kp = new double[Alice5ShmBridge.N_JOINTS];
   private final double[] kd = new double[Alice5ShmBridge.N_JOINTS];
   private JointDesiredOutputReadOnly[] jointDesiredOutputs = null;
   private long heartbeat = 0;
   private boolean dampingRequestSent = false;
   private volatile int lastDesiredPositionCount = -1;
   private long lastLoggedFaultBits = 0;

   public Alice5ShmCommunication(Alice5ShmBridge bridge, String[] orderedJointNames, double masterThreadDt)
   {
      if (orderedJointNames.length != Alice5ShmBridge.N_JOINTS)
         throw new IllegalArgumentException("Expected " + Alice5ShmBridge.N_JOINTS + " joints, got " + orderedJointNames.length);

      this.bridge = bridge;
      this.orderedJointNames = orderedJointNames.clone();
      this.jointStates = new LowLevelState[orderedJointNames.length];
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   @Override
   public void read(SensorDataContext sensorDataContext)
   {
      if (!bridge.isOpen())
         return;

      if (!contextRegistered || !sensorDataContext.isJointRegistered(orderedJointNames[0]))
         registerContext(sensorDataContext);

      long now = System.nanoTime();

      // Self-pause detection: if our own loop stalled longer than the freeze threshold,
      // treat it the same as a state freeze (stale internal state must not command).
      if (hasFirstStateVolatile && lastReadCallNs != 0 && now - lastReadCallNs > FREEZE_THRESHOLD_NS)
         stateFrozen.set(true);
      lastReadCallNs = now;

      boolean readOk = bridge.readState(scratchSnapshot);
      if (readOk)
      {
         if (!hasSnapshot || scratchSnapshot.seq != latestSnapshot.seq)
            lastSeqChangeNs = now;
         latestSnapshot.set(scratchSnapshot);
         hasSnapshot = true;
      }
      else
      {
         staleReadExhaustedCount.increment();
      }

      if (!hasSnapshot)
         return;

      // Staleness watchdog on the state sequence number.
      if (hasFirstStateVolatile)
      {
         if (now - lastSeqChangeNs > FREEZE_THRESHOLD_NS)
         {
            stateFrozen.set(true);
         }
         else if (stateFrozen.getValue())
         {
            // State is fresh again: clear freeze. Re-engage is handled on the write
            // side by echoing measured q while the robot side reports DAMPING.
            stateFrozen.set(false);
            dampingRequestSent = false;
            LogTools.warn("State stream recovered, starting re-engage handshake.");
         }
      }

      updateFirstStateAcceptance(now);

      safetyState.set(latestSnapshot.safetyState);
      stateSequenceNumber.set(latestSnapshot.seq);
      if (latestSnapshot.faultBits != lastLoggedFaultBits)
      {
         LogTools.warn("Robot fault bits changed: {} -> {} (safety={})",
                       lastLoggedFaultBits, latestSnapshot.faultBits, Alice5ShmBridge.safetyStateName(latestSnapshot.safetyState));
         lastLoggedFaultBits = latestSnapshot.faultBits;
      }

      // The robot-side fault word is latched (bits are never cleared by the robot FSM), and the
      // INIT_REJECT / CMD_STALE bits describe recoverable handshake conditions the bridge is
      // designed to ride through (INIT echo handshake, DAMPING re-engage). Only terminal
      // conditions may trip the upstream fault listeners (which force DO_NOTHING and unservo
      // the robot): the robot side giving up (DISABLED) or a NaN command fault (blocks
      // re-engage, indicates a controller-side bug).
      boolean faultedNow = latestSnapshot.safetyState == Alice5ShmBridge.SAFETY_DISABLED
            || (latestSnapshot.faultBits & FAULT_NAN_BIT) != 0;
      robotFaulted.set(faultedNow);

      packSensorDataContext();
   }

   private void registerContext(SensorDataContext sensorDataContext)
   {
      // The estimator-side context may be pre-registered (SensorDataContext(fullRobotModel)).
      for (int i = 0; i < orderedJointNames.length; i++)
      {
         String name = orderedJointNames[i];
         jointStates[i] = sensorDataContext.isJointRegistered(name) ? sensorDataContext.getMeasuredJointState(name) : sensorDataContext.registerJoint(name);
      }
      imuData = sensorDataContext.isImuRegistered(Alice5SensorInformation.PELVIS_IMU) ? sensorDataContext.getImuMeasurement(Alice5SensorInformation.PELVIS_IMU)
            : sensorDataContext.registerImu(Alice5SensorInformation.PELVIS_IMU);
      footWrenches[0] = sensorDataContext.isForceSensorRegistered(LEFT_FOOT_FT_SENSOR) ? sensorDataContext.getForceSensorMeasurement(LEFT_FOOT_FT_SENSOR)
            : sensorDataContext.registerForceSensor(LEFT_FOOT_FT_SENSOR);
      footWrenches[1] = sensorDataContext.isForceSensorRegistered(RIGHT_FOOT_FT_SENSOR) ? sensorDataContext.getForceSensorMeasurement(RIGHT_FOOT_FT_SENSOR)
            : sensorDataContext.registerForceSensor(RIGHT_FOOT_FT_SENSOR);
      contextRegistered = true;
   }

   private void updateFirstStateAcceptance(long now)
   {
      if (hasFirstStateVolatile)
         return;

      if (firstSnapshotNs == 0)
      {
         firstSnapshotNs = now;
         firstObservedSeq = latestSnapshot.seq;
         firstObservedCycle = latestSnapshot.cycle;
         return;
      }

      // Accept only after observing seq and cycle advancing for 0.5 s (reject a frozen creator).
      if (now - firstSnapshotNs >= FIRST_STATE_OBSERVATION_NS && latestSnapshot.seq != firstObservedSeq && latestSnapshot.cycle != firstObservedCycle)
      {
         hasFirstStateVolatile = true;
         receivedFirstState.set(true);
         lastSeqChangeNs = now;
         LogTools.info("First valid state accepted: seq={}, cycle={}, safety={}",
                       latestSnapshot.seq, latestSnapshot.cycle, Alice5ShmBridge.safetyStateName(latestSnapshot.safetyState));
      }
   }

   private void packSensorDataContext()
   {
      for (int i = 0; i < jointStates.length; i++)
      {
         LowLevelState state = jointStates[i];
         state.setPosition(latestSnapshot.q[i]);
         state.setVelocity(latestSnapshot.qd[i]);
         state.setEffort(latestSnapshot.tau[i]);
      }

      // Shared memory stores w, x, y, z. IHMC quaternion set() takes x, y, z, w.
      tempOrientation.set(latestSnapshot.imuQuat[1], latestSnapshot.imuQuat[2], latestSnapshot.imuQuat[3], latestSnapshot.imuQuat[0]);
      imuData.setOrientation(tempOrientation);
      tempVector.set(latestSnapshot.imuGyro[0], latestSnapshot.imuGyro[1], latestSnapshot.imuGyro[2]);
      imuData.setAngularVelocity(tempVector);
      tempVector.set(latestSnapshot.imuAcc[0], latestSnapshot.imuAcc[1], latestSnapshot.imuAcc[2]);
      imuData.setLinearAcceleration(tempVector);

      // Shared memory foot F/T is force xyz then torque xyz. IHMC wrench convention is
      // angular (torque) first, then linear (force). The robot side reports the reaction the
      // foot applies to the ground (MuJoCo force-sensor convention: Fz ~ -mg/2 at stance);
      // IHMC expects the ground reaction on the foot (Fz ~ +mg/2, WrenchBasedFootSwitch tests
      // forceZUp > threshold), so the wrench is negated here.
      // SIM-EXT FT: gate the wrench on its validity bit. While valid, convert and cache it; when the
      // loadcell wrench is flagged invalid (CAN timeout, saturation, ...) hold the last valid wrench
      // and raise an alarm. Holding (not zeroing) keeps the foot switch correct for a foot that is
      // physically still in contact: zeroing would tell the switch the foot left the ground and could
      // destabilize the estimator. [HUMAN VERIFY] indefinite loss policy for real hardware (a foot
      // that genuinely lifts during a long dropout would be held in contact); the bridge staleness
      // watchdog still drops the whole stream to DAMPING when the state itself stops advancing.
      boolean anyStale = false;
      for (int side = 0; side < 2; side++)
      {
         DMatrixRMaj wrench = footWrenches[side];
         double[] held = lastValidWrench[side];
         boolean valid = (latestSnapshot.footFtValidity & WRENCH_VALID_BIT[side]) != 0;

         if (valid)
         {
            double[] ft = latestSnapshot.footFt[side];
            held[0] = -ft[3];
            held[1] = -ft[4];
            held[2] = -ft[5];
            held[3] = -ft[0];
            held[4] = -ft[1];
            held[5] = -ft[2];
            hasValidWrench[side] = true;
         }
         else if (!hasValidWrench[side])
         {
            // Never observed a valid wrench yet (e.g. invalid on the very first snapshot): there is
            // nothing to hold, so report zero. This only lasts until the first valid sample.
            for (int i = 0; i < 6; i++)
               held[i] = 0.0;
            anyStale = true;
         }
         else
         {
            anyStale = true;
         }

         for (int i = 0; i < 6; i++)
            wrench.set(i, 0, held[i]);
      }
      footWrenchStale.set(anyStale);
   }

   @Override
   public void write(JointDesiredOutputListReadOnly jointDesireds, double masterGain)
   {
      if (!bridge.isOpen() || !hasSnapshot)
         return;

      if (stateFrozen.getValue())
      {
         if (latestSnapshot.safetyState == Alice5ShmBridge.SAFETY_INIT)
         {
            // Not engaged yet: a freeze during the INIT handshake (e.g. master-loop stall on
            // the controller first tick) must not emit DAMPING_REQUEST, the robot side would
            // latch FAULT_INIT_REJECT. Keep echoing the measured configuration instead.
            heartbeat++;
            commandHeartbeat.set(heartbeat);
            echoMeasuredConfiguration();
            bridge.writeCommand(Alice5ShmBridge.MODE_POSITION, heartbeat, ALL_JOINTS_ENABLE_MASK, qDesired, qdDesired, tauFeedForward, kp, kd);
            return;
         }

         // Freeze: send a single DAMPING_REQUEST, then stop commanding (heartbeat halts,
         // the robot side watchdog keeps the robot in DAMPING regardless).
         if (!dampingRequestSent)
         {
            heartbeat++;
            commandHeartbeat.set(heartbeat);
            echoMeasuredConfiguration();
            bridge.writeCommand(Alice5ShmBridge.MODE_DAMPING_REQUEST, heartbeat, ALL_JOINTS_ENABLE_MASK, qDesired, qdDesired, tauFeedForward, kp, kd);
            dampingRequestSent = true;
            LogTools.warn("State frozen: sent DAMPING_REQUEST and halted command stream.");
         }
         return;
      }

      if (jointDesiredOutputs == null)
         mapJointDesiredOutputs(jointDesireds);

      heartbeat++;
      commandHeartbeat.set(heartbeat);

      // Echo the measured configuration while the robot side is still in INIT (engage
      // handshake) and during the DAMPING re-engage re-home. Once the robot side is NORMAL the
      // controller desireds (FREEZE hold at engage) flow immediately: echoing the measured
      // configuration any longer would leave the standing robot without postural support.
      boolean echoPhase = latestSnapshot.safetyState == Alice5ShmBridge.SAFETY_INIT
            || latestSnapshot.safetyState == Alice5ShmBridge.SAFETY_DAMPING;

      int desiredPositionCount = 0;

      for (int i = 0; i < Alice5ShmBridge.N_JOINTS; i++)
      {
         JointDesiredOutputReadOnly output = jointDesiredOutputs[i];

         if (output != null && output.hasDesiredPosition())
            desiredPositionCount++;

         if (echoPhase || output == null || !output.hasDesiredPosition())
            qDesired[i] = latestSnapshot.q[i];
         else
            qDesired[i] = output.getDesiredPosition();

         qdDesired[i] = (!echoPhase && output != null && output.hasDesiredVelocity()) ? output.getDesiredVelocity() : 0.0;
         tauFeedForward[i] = (!echoPhase && output != null && output.hasDesiredTorque()) ? masterGain * output.getDesiredTorque() : 0.0;
         // PD gains are sent at full strength: scaling them with the master gain leaves the
         // robot with stiffness below the inverted-pendulum bound for the whole servo ramp
         // (and bypasses the robot-side zero-gain hold), letting it collapse at engage. The
         // master gain ramp still applies to the torque feedforward above.
         kp[i] = (output != null && output.hasStiffness()) ? output.getStiffness() : 0.0;
         kd[i] = (output != null && output.hasDamping()) ? output.getDamping() : 0.0;
      }

      lastDesiredPositionCount = desiredPositionCount;

      bridge.writeCommand(Alice5ShmBridge.MODE_POSITION, heartbeat, ALL_JOINTS_ENABLE_MASK, qDesired, qdDesired, tauFeedForward, kp, kd);
   }

   public int getLastDesiredPositionCount()
   {
      return lastDesiredPositionCount;
   }

   private void echoMeasuredConfiguration()
   {
      for (int i = 0; i < Alice5ShmBridge.N_JOINTS; i++)
      {
         qDesired[i] = latestSnapshot.q[i];
         qdDesired[i] = 0.0;
         tauFeedForward[i] = 0.0;
         kp[i] = 0.0;
         kd[i] = 0.0;
      }
   }

   private void mapJointDesiredOutputs(JointDesiredOutputListReadOnly jointDesireds)
   {
      jointDesiredOutputs = new JointDesiredOutputReadOnly[Alice5ShmBridge.N_JOINTS];

      for (int abiIndex = 0; abiIndex < orderedJointNames.length; abiIndex++)
      {
         String jointName = orderedJointNames[abiIndex];
         for (int listIndex = 0; listIndex < jointDesireds.getNumberOfJointsWithDesiredOutput(); listIndex++)
         {
            if (jointName.equals(jointDesireds.getOneDoFJoint(listIndex).getName()))
            {
               jointDesiredOutputs[abiIndex] = jointDesireds.getJointDesiredOutput(listIndex);
               break;
            }
         }
         if (jointDesiredOutputs[abiIndex] == null)
            LogTools.warn("No desired output found for joint {}. Will echo measured position for it.", jointName);
      }
   }

   @Override
   public void start()
   {
      if (!bridge.isOpen())
         bridge.open();
   }

   @Override
   public void stop()
   {
   }

   @Override
   public void destroy()
   {
      bridge.close();
   }

   @Override
   public boolean hasReceivedFirstState()
   {
      return hasFirstStateVolatile;
   }

   @Override
   public boolean hasRobotFaulted()
   {
      return robotFaulted.getValue();
   }

   @Override
   public void addFaultListener(YoVariableChangedListener listener)
   {
      robotFaulted.addListener(listener);
   }

   public int getSafetyState()
   {
      return safetyState.getValue();
   }

   public boolean isStateFrozen()
   {
      return stateFrozen.getValue();
   }

   public boolean isFootWrenchStale()
   {
      return footWrenchStale.getValue();
   }
}
