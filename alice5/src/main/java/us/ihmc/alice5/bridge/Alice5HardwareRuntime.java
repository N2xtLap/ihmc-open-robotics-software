package us.ihmc.alice5.bridge;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import us.ihmc.alice5.Alice5RobotModel;
import us.ihmc.alice5.Alice5Version;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.wholeBodyHardwareControl.AvatarLowLevelOutputProcessor;
import us.ihmc.avatar.wholeBodyHardwareControl.AvatarMultiThreadingFactory;
import us.ihmc.avatar.wholeBodyHardwareControl.AvatarMultiThreadingManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.FreezeControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StandPrepControllerStateFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.log.LogTools;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * Hardware runtime entry point for ALICE5: estimator + walking controller wired to the
 * shared-memory bridge (/dev/shm/alice5_wbc) instead of SCS.
 * <p>
 * Assembly uses AvatarMultiThreadingFactory with useRealtimeThreads=false and
 * useMultiThreading=false (single-threaded master loop at the estimator period).
 * <p>
 * Exit codes: 0 normal completion, 4 shared-memory open/validation failure.
 * System properties: -Dalice5.runtime.duration=&lt;seconds&gt; (default 30),
 * -Dalice5.rtstats=&lt;true|false&gt; (default true: RT health diagnostics, SIM-EXT RT-OBS).
 */
public class Alice5HardwareRuntime
{
   private static final double STAND_PREP_TO_TRANSITION_DELAY_S = 6.0;
   private static final double STANDING_ROOT_Z_THRESHOLD = 0.7;
   private static final int STANDING_PASS_SECONDS = 60;

   private static final AtomicReference<Enum<?>> currentControllerState = new AtomicReference<>(null);
   private static final AtomicInteger controllerTickCount = new AtomicInteger(0);

   // Gate metrics: consecutive standing seconds (rootZ above threshold, safety NORMAL, no
   // freeze), updated by the 1 Hz status logger and reported at shutdown.
   private static final Object gateLock = new Object();
   private static int standingStreakSeconds = 0;
   private static double standingStreakMinRootZ = Double.POSITIVE_INFINITY;

   // SIM-EXT FT: foot-wrench stale-hold observation while the loadcell validity bit drops.
   private static boolean footWrenchEverStale = false;
   private static double footWrenchStaleMinRootZ = Double.POSITIVE_INFINITY;
   private static int footWrenchStaleSamples = 0;

   public static void main(String[] args)
   {
      double durationSeconds = Double.parseDouble(System.getProperty("alice5.runtime.duration", "30.0"));

      Alice5RobotModel robotModel = new Alice5RobotModel(Alice5Version.V1_FULL_ROBOT, RobotTarget.REAL_ROBOT);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      Alice5ShmBridge bridge = new Alice5ShmBridge();
      try
      {
         bridge.open();
      }
      catch (RuntimeException e)
      {
         LogTools.error("Shared memory bridge open failed: {}", e.getMessage());
         System.exit(4);
      }

      double masterThreadDt = robotModel.getEstimatorDT();
      YoRegistry rootRegistry = new YoRegistry("Alice5HardwareRuntime");

      Alice5ShmCommunication communication = new Alice5ShmCommunication(bridge, robotModel.getJointMap().getOrderedJointNames(), masterThreadDt);
      rootRegistry.addChild(communication.getRegistry());

      // RT health diagnostics (SIM-EXT RT-OBS): best-effort, never touches the control path.
      boolean rtStatsEnabled = Boolean.parseBoolean(System.getProperty("alice5.rtstats", "true"));
      Alice5RtStats rtStats = rtStatsEnabled ? new Alice5RtStats(robotModel.getControllerDT()) : null;
      if (rtStats != null)
         rootRegistry.addChild(rtStats.getRegistry());

      Alice5ShmSensorReaderFactory sensorReaderFactory = new Alice5ShmSensorReaderFactory(robotModel.getStateEstimatorParameters());

      YoVariableServer yoVariableServer = new YoVariableServer("Alice5HardwareRuntime",
                                                               robotModel.getLogModelProvider(),
                                                               new DataServerSettings(false),
                                                               masterThreadDt);

      AvatarMultiThreadingFactory factory = new AvatarMultiThreadingFactory(robotModel,
                                                                            fullRobotModel,
                                                                            communication,
                                                                            sensorReaderFactory,
                                                                            new StandPrepControllerStateFactory(),
                                                                            new FreezeControllerStateFactory(),
                                                                            new Alice5NoOpAffinity(),
                                                                            true, // createStepGeneratorThread (false crashes upstream: OptionalFactoryField.get() on unset field)
                                                                            false, // useRealtimeThreads
                                                                            true, // useMultiThreading: one-time controller-state initialization (walking core setup)
                                                                                  // takes greater than 20 ms; inline (single-threaded) it stalls the 1 kHz master loop and
                                                                                  // heartbeat, which makes the robot side latch CMD_STALE and drop to DAMPING.
                                                                            new MonotonicTime(0, 1_000_000L), // 1 ms period
                                                                            masterThreadDt,
                                                                            System::nanoTime,
                                                                            rootRegistry,
                                                                            yoVariableServer);

      AvatarMultiThreadingManager manager = factory.buildThreadsAndThreadingManager();

      HighLevelHumanoidControllerFactory controllerFactory = factory.getHighLevelHumanoidControllerFactory();
      AvatarLowLevelOutputProcessor outputProcessor = factory.getLowLevelOutputProcessor();
      YoEnum<HighLevelControllerName> requestedState = controllerFactory.getRequestedControlStateEnum();

      controllerFactory.attachControllerStateChangedListener((oldState, newState) ->
      {
         currentControllerState.set(newState);
         LogTools.info("High level controller state changed: {} -> {}", oldState, newState);
      });

      controllerFactory.attachControllerFailureListener(fallingDirection -> LogTools.warn("Controller failure reported, falling direction: {}",
                                                                                          fallingDirection));

      // Unfreeze the state estimator already during STAND_PREP / STAND_READY (the factory default
      // keeps it FROZEN until STAND_TRANSITION). Unfreezing snaps the pelvis state to be consistent
      // with the feet, which causes a CoM-velocity/ICP transient; in STAND_PREP no failure detection
      // is active so the estimator has settled by the time the walking failure detector starts.
      // This listener is attached after the factory default one, so its request wins for these states.
      Map<HighLevelControllerName, StateEstimatorMode> estimatorModeOverrides = new HashMap<>();
      estimatorModeOverrides.put(HighLevelControllerName.STAND_PREP_STATE, StateEstimatorMode.NORMAL);
      estimatorModeOverrides.put(HighLevelControllerName.STAND_READY, StateEstimatorMode.NORMAL);
      factory.getEstimatorThread().setupHighLevelControllerCallback(controllerFactory, estimatorModeOverrides);

      // The default 5 s master-gain servo ramp leaves the robot with low PD gains for seconds
      // (the robot-side PD fallback only applies at exactly zero gains), letting it sag under
      // gravity. In simulation the gains can come up fast: ramp over 0.5 s.
      YoDouble servoDuration = (YoDouble) rootRegistry.findVariable("servoDuration");
      if (servoDuration != null)
         servoDuration.set(0.5);

      manager.addPostControllerThreadRunnable(controllerTickCount::incrementAndGet);

      if (rtStats != null)
      {
         // ControllerTask's ThreadTimer ("ControllerTimer", ms) is the tick duration source;
         // ControllerTask only supports post-task callbacks (pre-task throws upstream). The
         // controller thread registry is NOT under rootRegistry (it is registered straight
         // with the YoVariableServer), so look it up through the factory.
         rtStats.setControllerTickTimer((YoDouble) factory.getControllerRegistry().findVariable("ControllerTimer"));
         manager.addPostControllerThreadRunnable(rtStats::onControllerTickEnd);
      }

      try
      {
         yoVariableServer.start();
      }
      catch (RuntimeException e)
      {
         LogTools.warn("YoVariableServer failed to start (continuing without it): {}", e.getMessage());
      }

      factory.start();

      long startTimeNs = System.nanoTime();

      Thread sequencer = new Thread(() -> runStateSequencer(communication, outputProcessor, requestedState), "Alice5StateSequencer");
      sequencer.setDaemon(true);
      sequencer.start();

      Thread statusLogger = new Thread(() -> runStatusLogger(communication, outputProcessor, fullRobotModel, startTimeNs, rtStats), "Alice5StatusLogger");
      statusLogger.setDaemon(true);
      statusLogger.start();

      try
      {
         Thread.sleep((long) (durationSeconds * 1000.0));
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
      }

      reportGateMetrics();

      LogTools.info("Run duration of {} s elapsed, shutting down.", durationSeconds);
      try
      {
         factory.destroy();
      }
      catch (RuntimeException e)
      {
         // The upstream non-realtime multi-threaded teardown path is imperfect (plain Threads
         // are cast to RepeatingTaskThread); the gate metrics are already reported, exit clean.
         LogTools.warn("Exception during shutdown (ignored): {}", e.toString());
      }
      System.exit(0);
   }

   private static void reportGateMetrics()
   {
      int standing;
      double rootZMin;
      synchronized (gateLock)
      {
         standing = standingStreakSeconds;
         rootZMin = standingStreakMinRootZ;
      }

      System.out.println(String.format("GATE_M3_METRICS standing=%d rootZmin=%.4f", standing, rootZMin));
      if (standing >= STANDING_PASS_SECONDS)
         System.out.println("GATE_M3_STANDING_PASS");

      System.out.println(String.format("GATE_FT_METRICS footWrenchEverStale=%b staleSamples=%d staleMinRootZ=%.4f",
                                        footWrenchEverStale, footWrenchStaleSamples, footWrenchStaleMinRootZ));
   }

   private static void runStateSequencer(Alice5ShmCommunication communication,
                                         AvatarLowLevelOutputProcessor outputProcessor,
                                         YoEnum<HighLevelControllerName> requestedState)
   {
      try
      {
         while (!communication.hasReceivedFirstState())
            Thread.sleep(100L);

         // Servo right away: until the master gain is up the robot only has the low-level
         // fallback hold, so the controller gains should arrive as soon as possible.
         outputProcessor.servoRobot();

         // Make sure the controller thread is actually ticking and the engage handshake has
         // settled before requesting transitions.
         while (controllerTickCount.get() < 10)
            Thread.sleep(100L);
         Thread.sleep(1000L);

         for (int attempt = 1; attempt <= 3; attempt++)
         {
            if (runStandingLadder(outputProcessor, requestedState))
            {
               LogTools.info("Standing ladder complete (attempt {}).", attempt);
               return;
            }
            LogTools.warn("Standing ladder attempt {} failed (current controller state: {}).", attempt, currentControllerState.get());
         }
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
      }
   }

   private static boolean runStandingLadder(AvatarLowLevelOutputProcessor outputProcessor, YoEnum<HighLevelControllerName> requestedState)
         throws InterruptedException
   {
      // If a fault pushed the controller to DO_NOTHING, recover through FREEZE
      // (DO_NOTHING -> STAND_PREP is not a registered transition, DO_NOTHING -> FREEZE is).
      if (currentControllerState.get() == HighLevelControllerName.DO_NOTHING_BEHAVIOR)
      {
         if (!requestUntil(requestedState,
                           HighLevelControllerName.FREEZE_STATE,
                           () -> currentControllerState.get() == HighLevelControllerName.FREEZE_STATE,
                           10.0))
            return false;
      }

      // Servo (or re-servo after a fault-driven unservo): ramps the master gain to 1 over 5 s.
      LogTools.info("Starting servo ramp (master gain {} -> 1).", outputProcessor.getMasterGain().getValue());
      outputProcessor.servoRobot();
      long servoDeadlineNs = System.nanoTime() + 10_000_000_000L;
      while (outputProcessor.getMasterGain().getValue() < 0.999 && System.nanoTime() < servoDeadlineNs)
         Thread.sleep(100L);
      if (outputProcessor.getMasterGain().getValue() < 0.999)
      {
         LogTools.warn("Servo ramp did not complete (master gain = {}).", outputProcessor.getMasterGain().getValue());
         return false;
      }
      LogTools.info("Servo ramp finished: master gain = {}.", outputProcessor.getMasterGain().getValue());

      if (!requestUntil(requestedState,
                        HighLevelControllerName.STAND_PREP_STATE,
                        () -> currentControllerState.get() == HighLevelControllerName.STAND_PREP_STATE,
                        10.0))
         return false;

      Thread.sleep((long) (STAND_PREP_TO_TRANSITION_DELAY_S * 1000.0));

      if (!requestUntil(requestedState,
                        HighLevelControllerName.STAND_TRANSITION_STATE,
                        () -> isStandingState(currentControllerState.get()),
                        10.0))
         return false;

      // Confirm the controller holds a standing state (no immediate failure fallback to FREEZE).
      long observeDeadlineNs = System.nanoTime() + 10_000_000_000L;
      while (System.nanoTime() < observeDeadlineNs)
      {
         if (!isStandingState(currentControllerState.get()))
            return false;
         Thread.sleep(500L);
      }
      return true;
   }

   private static boolean isStandingState(Enum<?> state)
   {
      if (state == null)
         return false;
      String name = state.toString();
      return name.equals("STAND_TRANSITION_STATE") || name.equals("STAND_READY") || name.equals("WALKING");
   }

   private static boolean requestUntil(YoEnum<HighLevelControllerName> requestedState,
                                       HighLevelControllerName target,
                                       BooleanSupplier reached,
                                       double timeoutSeconds)
         throws InterruptedException
   {
      LogTools.info("Requesting {}.", target);
      long deadlineNs = System.nanoTime() + (long) (timeoutSeconds * 1.0e9);
      while (!reached.getAsBoolean())
      {
         if (System.nanoTime() > deadlineNs)
         {
            LogTools.warn("Timed out waiting to reach {} (current controller state: {}).", target, currentControllerState.get());
            return false;
         }
         requestedState.set(target);
         Thread.sleep(500L);
      }
      LogTools.info("Reached {} (current controller state: {}).", target, currentControllerState.get());
      return true;
   }

   private static void runStatusLogger(Alice5ShmCommunication communication,
                                       AvatarLowLevelOutputProcessor outputProcessor,
                                       FullHumanoidRobotModel fullRobotModel,
                                       long startTimeNs,
                                       Alice5RtStats rtStats)
   {
      try
      {
         while (true)
         {
            Thread.sleep(1000L);

            double elapsed = (System.nanoTime() - startTimeNs) / 1.0e9;
            int safetyState = communication.getSafetyState();
            String safetyName = Alice5ShmBridge.safetyStateName(safetyState);
            boolean frozen = communication.isStateFrozen();
            boolean footWrenchStale = communication.isFootWrenchStale();
            Enum<?> hlcState = currentControllerState.get();
            String hlcName = hlcState == null ? "n/a" : hlcState.toString();
            double rootZ = fullRobotModel.getRootJoint().getJointPose().getZ();

            if (footWrenchStale)
            {
               footWrenchEverStale = true;
               footWrenchStaleSamples++;
               footWrenchStaleMinRootZ = Math.min(footWrenchStaleMinRootZ, rootZ);
            }

            synchronized (gateLock)
            {
               // Standing requires the estimator to actually run (NORMAL estimator mode), which
               // only happens from STAND_TRANSITION onward. This keeps a frozen-estimator rootZ
               // from trivially passing the gate.
               boolean standingSample = isStandingState(hlcState) && rootZ > STANDING_ROOT_Z_THRESHOLD && safetyState == Alice5ShmBridge.SAFETY_NORMAL && !frozen;
               if (standingSample)
               {
                  standingStreakSeconds++;
                  standingStreakMinRootZ = Math.min(standingStreakMinRootZ, rootZ);
               }
               else
               {
                  standingStreakSeconds = 0;
                  standingStreakMinRootZ = Double.POSITIVE_INFINITY;
               }
            }

            String rtSummary = rtStats == null ? "" : " " + rtStats.updateAndSummarize(frozen);

            System.out.println(String.format("RUNTIME t=%.1f state=%s hlc=%s rootZ(estimator pelvis z)=%.4f masterGain=%.2f desiredPosJoints=%d controllerTicks=%d frozen=" + frozen + " footWrenchStale=" + footWrenchStale + " streak=" + standingStreakSeconds + rtSummary,
                                             elapsed,
                                             safetyName,
                                             hlcName,
                                             rootZ,
                                             outputProcessor.getMasterGain().getValue(),
                                             communication.getLastDesiredPositionCount(),
                                             controllerTickCount.get()));
         }
      }
      catch (InterruptedException e)
      {
         Thread.currentThread().interrupt();
      }
   }
}
