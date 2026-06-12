package us.ihmc.alice5.teleop;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;

/**
 * SIM-EXT V1: YoVariableClient wrapper for ALICE5 teleop.
 *
 * Connects to the Alice5WalkingDemo YoVariableServer (joystick CSG mode), resolves the
 * StepGeneratorCommandInputManager input variables plus the estimator root-joint state variables,
 * and pushes velocity commands through the server's variable-change channel.
 *
 * How the change channel works (verified against ihmc-robot-data-logger 0.36.4):
 * <ul>
 * <li>changesVariables()=true makes the client register a VariableChangedProducer listener on every
 * mirror variable; a local set() that changes the value sends a change request to the server.</li>
 * <li>The server queues change requests (YoVariableServer.changeVariable) and applies them inside
 * YoVariableServer.update(), which the SCS2AvatarSimulationFactory wires as a post-task callback of
 * the owning thread (stepGeneratorTask for the CSG inputs) -- i.e. writes land on the controller
 * side thread-safely, unlike main-thread writes inside the demo process (lost, see M2 notes).</li>
 * <li>Incoming stream data is decompressed into the mirror registry with notifyListeners=false
 * (RegistryDecompressor), so streaming never echoes change requests back. It does silently
 * overwrite local mirrors with server values, therefore commands must be re-asserted every control
 * tick: set() only fires when the command differs from the latest streamed server value.</li>
 * </ul>
 */
public class TeleopYoVariableConnection implements YoVariablesUpdatedListener
{
   public static final int DEFAULT_PORT = DataServerSettings.DEFAULT_PORT;

   private final YoVariableClient client = new YoVariableClient(this);
   private final CountDownLatch handshakeLatch = new CountDownLatch(1);
   private final AtomicLong packetCount = new AtomicLong();
   private final AtomicLong lastDataWallNanos = new AtomicLong();
   private final AtomicLong latestTimestampNanos = new AtomicLong();

   private YoVariableClientInterface clientInterface;

   // command variables (StepGeneratorCommandInputManager, stepGenerator thread)
   private YoBoolean walk;
   private YoDouble desiredVelocityX;
   private YoDouble desiredVelocityY;
   private YoDouble desiredTurningVelocity;

   // state variables. Preferred source: estimator estimatedRootJoint* (kinematics-based state
   // estimator). With usePerfectSensors=true those do not exist, so fall back to the controller
   // thread's CommonHumanoidReferenceFramesVisualizer afterPelvis pose (world frame, every tick).
   private YoVariable posX, posY, posZ;
   private YoVariable yaw; // direct yaw variable if available, otherwise NaN (see quaternion below)
   private YoVariable quatQx, quatQy, quatQz, quatQs; // afterPelvis quaternion fallback
   private YoVariable velX, velY;
   private YoVariable angVelZ;
   private YoVariable walkingState;

   // CSG parameters for physical-unit -> unit-fraction conversion. In this CSG version the
   // non-unit-velocity input branch computes effective = minMaxVelocity * clamp(input, 1.0)
   // (ContinuousStepGenerator.update, confirmed in M2): the raw input is a FRACTION of the max
   // velocity regardless of the isUnitVelocities flag. The client therefore converts m/s and
   // rad/s commands to fractions using the published CSG parameters.
   private YoVariable swingTimeCSG, transferTimeCSG;
   private YoVariable maxStepLengthCSG, maxStepWidthCSG;
   private YoVariable maxAngleTurnInwardsCSG, maxAngleTurnOutwardsCSG;
   // effective (post-scaling) velocities actually used by the CSG, for verification
   private YoVariable csgVelocityX, csgVelocityY, csgTurningVelocity;

   public void connect(String host, int port, double timeoutSeconds) throws Exception
   {
      System.out.println("[teleop] connecting to " + host + ":" + port);
      client.start(host, port);
      if (!handshakeLatch.await((long) (timeoutSeconds * 1000.0), TimeUnit.MILLISECONDS))
         throw new RuntimeException("YoVariableClient handshake timed out after " + timeoutSeconds + " s");
   }

   public void disconnect()
   {
      try
      {
         client.stop();
      }
      catch (Exception e)
      {
         System.out.println("[teleop] disconnect: " + e);
      }
   }

   /**
    * Re-assert the command every control tick. set() only sends a change request when the value
    * differs from the latest streamed server value, so steady-state traffic is zero.
    */
   public void sendCommand(boolean walkCommand, double vx, double vy, double wz)
   {
      walk.set(walkCommand);
      desiredVelocityX.set(vx);
      desiredVelocityY.set(vy);
      desiredTurningVelocity.set(wz);
   }

   /** Immediate stop: walk=false and zero velocities (E-stop path, no ramping). */
   public void sendEStop()
   {
      sendCommand(false, 0.0, 0.0, 0.0);
   }

   /**
    * Command in physical units (m/s, rad/s): converts to unit fractions of the CSG max velocities
    * (see field comment) before writing. Values beyond the CSG limits saturate.
    */
   public void sendCommandPhysical(boolean walkCommand, double vxMetersPerSecond, double vyMetersPerSecond, double wzRadiansPerSecond)
   {
      sendCommand(walkCommand,
                  toFraction(vxMetersPerSecond, getMaxVelocityX()),
                  toFraction(vyMetersPerSecond, getMaxVelocityY()),
                  toFraction(compensateTurnRate(wzRadiansPerSecond), getMaxVelocityTurn()));
   }

   /**
    * The CSG clamps the per-step heading displacement d = stepTime * wz per swing side
    * (ContinuousStepGenerator.calculateNextFootstepPose2D): the outward-swing step turns
    * clamp(d, in, out), the inward-swing step only clamp(d, -out, -in). With ALICE5's runtime
    * turnMaxAngleInward = 0 only every other step turns, so the achieved average yaw rate is half
    * the effective command (measured: cmd 0.3 -> 0.15 rad/s). Invert that model so a physical
    * wz request yields the requested average rate: d = 2*a + in for a > -in, capped at out.
    * Max achievable average rate = (out - in) / (2 * stepTime).
    */
   private double compensateTurnRate(double wzRadiansPerSecond)
   {
      double stepTime = getStepTime();
      if (Double.isNaN(stepTime) || maxAngleTurnInwardsCSG == null || maxAngleTurnOutwardsCSG == null || wzRadiansPerSecond == 0.0)
         return wzRadiansPerSecond;
      double in = maxAngleTurnInwardsCSG.getValueAsDouble();
      double out = maxAngleTurnOutwardsCSG.getValueAsDouble();
      double targetPerStep = Math.abs(wzRadiansPerSecond) * stepTime;
      double commandedPerStep = targetPerStep <= -in ? targetPerStep : 2.0 * targetPerStep + in;
      commandedPerStep = Math.min(commandedPerStep, out);
      return Math.signum(wzRadiansPerSecond) * commandedPerStep / stepTime;
   }

   private static double toFraction(double value, double maxVelocity)
   {
      if (Double.isNaN(maxVelocity) || maxVelocity <= 0.0)
         return value; // conversion unavailable: pass through
      return Math.max(-1.0, Math.min(1.0, value / maxVelocity));
   }

   private double getStepTime()
   {
      if (swingTimeCSG == null || transferTimeCSG == null)
         return Double.NaN;
      return swingTimeCSG.getValueAsDouble() + transferTimeCSG.getValueAsDouble();
   }

   /** Max forward velocity [m/s] = maxStepLength / stepTime. */
   public double getMaxVelocityX()
   {
      return maxStepLengthCSG == null ? Double.NaN : maxStepLengthCSG.getValueAsDouble() / getStepTime();
   }

   /** Max lateral velocity [m/s] = maxStepWidth / stepTime. */
   public double getMaxVelocityY()
   {
      return maxStepWidthCSG == null ? Double.NaN : maxStepWidthCSG.getValueAsDouble() / getStepTime();
   }

   /** Max turning velocity [rad/s] = (maxAngleTurnOutwards - maxAngleTurnInwards) / stepTime. */
   public double getMaxVelocityTurn()
   {
      if (maxAngleTurnInwardsCSG == null || maxAngleTurnOutwardsCSG == null)
         return Double.NaN;
      return (maxAngleTurnOutwardsCSG.getValueAsDouble() - maxAngleTurnInwardsCSG.getValueAsDouble()) / getStepTime();
   }

   /** Effective forward velocity [m/s] the CSG is using (post-scaling), for verification. */
   public double getCsgEffectiveVelocityX()
   {
      return csgVelocityX == null ? Double.NaN : csgVelocityX.getValueAsDouble();
   }

   public double getCsgEffectiveVelocityY()
   {
      return csgVelocityY == null ? Double.NaN : csgVelocityY.getValueAsDouble();
   }

   public double getCsgEffectiveTurningVelocity()
   {
      return csgTurningVelocity == null ? Double.NaN : csgTurningVelocity.getValueAsDouble();
   }

   public boolean hasCommandVariables()
   {
      return walk != null && desiredVelocityX != null && desiredVelocityY != null && desiredTurningVelocity != null;
   }

   public boolean hasStateVariables()
   {
      boolean yawAvailable = yaw != null || (quatQx != null && quatQy != null && quatQz != null && quatQs != null);
      return posX != null && posY != null && posZ != null && yawAvailable;
   }

   public boolean getWalkReadback()
   {
      return walk != null && walk.getBooleanValue();
   }

   public double getX()
   {
      return posX == null ? Double.NaN : posX.getValueAsDouble();
   }

   public double getY()
   {
      return posY == null ? Double.NaN : posY.getValueAsDouble();
   }

   public double getZ()
   {
      return posZ == null ? Double.NaN : posZ.getValueAsDouble();
   }

   public double getYaw()
   {
      if (yaw != null)
         return yaw.getValueAsDouble();
      if (quatQx == null || quatQy == null || quatQz == null || quatQs == null)
         return Double.NaN;
      double qx = quatQx.getValueAsDouble();
      double qy = quatQy.getValueAsDouble();
      double qz = quatQz.getValueAsDouble();
      double qs = quatQs.getValueAsDouble();
      return Math.atan2(2.0 * (qs * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
   }

   public double getEstimatedVelocityX()
   {
      return velX == null ? Double.NaN : velX.getValueAsDouble();
   }

   public double getEstimatedVelocityY()
   {
      return velY == null ? Double.NaN : velY.getValueAsDouble();
   }

   public double getEstimatedYawRate()
   {
      return angVelZ == null ? Double.NaN : angVelZ.getValueAsDouble();
   }

   public String getWalkingState()
   {
      return walkingState == null ? "n/a" : walkingState.getValueAsString();
   }

   public long getPacketCount()
   {
      return packetCount.get();
   }

   /** Seconds since the last streamed data packet arrived (liveness). */
   public double getDataAgeSeconds()
   {
      long last = lastDataWallNanos.get();
      if (last == 0)
         return Double.POSITIVE_INFINITY;
      return (System.nanoTime() - last) / 1e9;
   }

   public boolean isConnected()
   {
      return clientInterface != null && clientInterface.isConnected();
   }

   // ------------------------------------------------------------------
   // YoVariablesUpdatedListener
   // ------------------------------------------------------------------

   @Override
   public boolean updateYoVariables()
   {
      return true;
   }

   @Override
   public boolean changesVariables()
   {
      return true;
   }

   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake,
                     YoVariableHandshakeParser handshakeParser, DebugRegistry debugRegistry)
   {
      this.clientInterface = yoVariableClientInterface;
      YoRegistry root = handshakeParser.getRootRegistry();

      walk = (YoBoolean) findExact(root, "walk_StepGeneratorCommandInputManager");
      desiredVelocityX = (YoDouble) findExact(root, "desiredVelocity_StepGeneratorCommandInputManagerX");
      desiredVelocityY = (YoDouble) findExact(root, "desiredVelocity_StepGeneratorCommandInputManagerY");
      desiredTurningVelocity = (YoDouble) findExact(root, "desiredTurningVelocity_StepGeneratorCommandInputManager");

      posX = findExact(root, "estimatedRootJointPositionX");
      posY = findExact(root, "estimatedRootJointPositionY");
      posZ = findExact(root, "estimatedRootJointPositionZ");
      yaw = findExact(root, "estimatedRootJointYaw");
      velX = findExact(root, "estimatedRootJointLinearVelocityX");
      velY = findExact(root, "estimatedRootJointLinearVelocityY");
      angVelZ = findExact(root, "estimatedRootJointAngularVelocityWorldZ");
      if (posX == null || posY == null || posZ == null)
      {
         // perfect-sensors config: use the controller-thread world pelvis pose instead
         posX = findExact(root, "afterPelvisX");
         posY = findExact(root, "afterPelvisY");
         posZ = findExact(root, "afterPelvisZ");
      }
      if (yaw == null)
      {
         quatQx = findExact(root, "afterPelvisQx");
         quatQy = findExact(root, "afterPelvisQy");
         quatQz = findExact(root, "afterPelvisQz");
         quatQs = findExact(root, "afterPelvisQs");
      }
      walkingState = findBySuffix(root, "walkingCurrentState");

      swingTimeCSG = findExact(root, "swingTimeCSG");
      transferTimeCSG = findExact(root, "transferTimeCSG");
      maxStepLengthCSG = findExact(root, "maxStepLengthCSG");
      maxStepWidthCSG = findExact(root, "maxStepWidthCSG");
      maxAngleTurnInwardsCSG = findExact(root, "maxAngleTurnInwardsCSG");
      maxAngleTurnOutwardsCSG = findExact(root, "maxAngleTurnOutwardsCSG");
      csgVelocityX = findExact(root, "desiredVelocityCSGX");
      csgVelocityY = findExact(root, "desiredVelocityCSGY");
      csgTurningVelocity = findExact(root, "desiredTurningVelocityCSG");

      System.out.println("[teleop] handshake with '" + yoVariableClientInterface.getServerName() + "' complete. Resolved variables:");
      System.out.println("[teleop]   walk            = " + name(walk));
      System.out.println("[teleop]   desiredVelocityX= " + name(desiredVelocityX));
      System.out.println("[teleop]   desiredVelocityY= " + name(desiredVelocityY));
      System.out.println("[teleop]   turningVelocity = " + name(desiredTurningVelocity));
      System.out.println("[teleop]   posX/Y/Z        = " + name(posX) + " / " + name(posY) + " / " + name(posZ));
      System.out.println("[teleop]   yaw             = " + (yaw != null ? name(yaw) : "from quaternion " + name(quatQs)));
      System.out.println("[teleop]   velX/Y          = " + name(velX) + " / " + name(velY));
      System.out.println("[teleop]   yawRate         = " + name(angVelZ));
      System.out.println("[teleop]   walkingState    = " + name(walkingState));
      System.out.println("[teleop]   csg params      = swing/transfer/maxStepLength/maxStepWidth/turnIn/turnOut: "
            + name(swingTimeCSG) + " (and siblings)");

      if (!hasStateVariables())
         dumpCandidates(root, "estimatedRootJoint");

      handshakeLatch.countDown();
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
      packetCount.incrementAndGet();
      lastDataWallNanos.set(System.nanoTime());
      latestTimestampNanos.set(timestamp);
   }

   /**
    * Latest robot-side timestamp (nanoseconds, simulation clock). The sim may run slower than wall
    * clock (Jetson 201: ~0.35x), so command scheduling and velocity measurements must use this.
    */
   public long getLatestTimestampNanos()
   {
      return latestTimestampNanos.get();
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
   }

   @Override
   public void connected()
   {
      System.out.println("[teleop] connected");
   }

   @Override
   public void disconnected()
   {
      System.out.println("[teleop] disconnected");
   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
   }

   // ------------------------------------------------------------------

   private static String name(YoVariable variable)
   {
      return variable == null ? "MISSING" : variable.getFullNameString();
   }

   private static YoVariable findExact(YoRegistry registry, String variableName)
   {
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         if (variable.getName().equals(variableName))
            return variable;
      }
      return null;
   }

   private static YoVariable findBySuffix(YoRegistry registry, String suffix)
   {
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         if (variable.getName().endsWith(suffix))
            return variable;
      }
      return null;
   }

   private static void dumpCandidates(YoRegistry registry, String nameFragment)
   {
      System.out.println("[teleop] state variables missing -- candidates containing '" + nameFragment + "':");
      List<YoVariable> all = registry.collectSubtreeVariables();
      int shown = 0;
      for (YoVariable variable : all)
      {
         if (variable.getName().contains(nameFragment) && shown < 40)
         {
            System.out.println("[teleop]   " + variable.getFullNameString());
            shown++;
         }
      }
   }
}
