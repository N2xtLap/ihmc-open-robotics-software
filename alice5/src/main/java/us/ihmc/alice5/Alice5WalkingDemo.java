package us.ihmc.alice5;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.wholeBodyController.RobotContactPointParameters.GroundContactModelParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * M2 gate demo: headless SCS2 flat-ground WALKING via ContinuousStepGenerator.
 *
 * Scenario: settle (5 s) -> walkCSG=true with commanded forward velocity -> walk until duration
 * (optional push at alice5.push.time). Metrics: steps (state-transition listener), distance/speed
 * (sim ground truth), controller tick compute p50/p99 (controllerThreadTimerCurrent listener).
 *
 * SIM-EXT V1 additions (both opt-in, the M2 gate path is untouched when they are off):
 * <ul>
 * <li>{@code -Dalice5.csg=joystick}: use the joystick stepping plugin
 * ({@code setDefaultHighLevelHumanoidControllerFactory(false, null)}) instead of the
 * heading-and-velocity script. CSG inputs are then driven at runtime through the YoVariableServer
 * variable-change channel ({@code walk_StepGeneratorCommandInputManager} and friends) by a remote
 * {@code us.ihmc.alice5.teleop} client. Requires {@code -Dcreate.yovariable.server=true}.</li>
 * <li>{@code -Dalice5.realtime=true}: pace the simulation at 1.0x wall clock
 * ({@code setRealTimeRateSimulation(true)} + asynchronous {@code simulate()}) instead of the
 * synchronous {@code simulateNow} gate loop. The main thread only monitors for falls.</li>
 * </ul>
 */
public class Alice5WalkingDemo
{
   public static void main(String[] args)
   {
      double duration = Double.parseDouble(System.getProperty("alice5.duration", "60.0"));
      double settleTime = Double.parseDouble(System.getProperty("alice5.settle", "5.0"));
      double velocity = Double.parseDouble(System.getProperty("alice5.vel", "0.125"));
      double swingTime = Double.parseDouble(System.getProperty("alice5.swing", "0.8"));
      double transferTime = Double.parseDouble(System.getProperty("alice5.transfer", "0.4"));
      double maxStepLength = Double.parseDouble(System.getProperty("alice5.maxStepLength", "0.35"));
      int minSteps = Integer.parseInt(System.getProperty("alice5.minSteps", "20"));
      double minSpeed = Double.parseDouble(System.getProperty("alice5.minSpeed", "0.0"));
      double pushTime = Double.parseDouble(System.getProperty("alice5.push.time", "-1"));
      double pushForce = Double.parseDouble(System.getProperty("alice5.push.force", "50"));
      double pushDuration = Double.parseDouble(System.getProperty("alice5.push.duration", "0.1"));
      String csvPath = System.getProperty("alice5.metrics.csv", "alice5_m2_metrics.csv");
      boolean createYoVariableServer = Boolean.parseBoolean(System.getProperty("create.yovariable.server", "false"));
      boolean joystickCsg = "joystick".equalsIgnoreCase(System.getProperty("alice5.csg", "script"));
      boolean realtime = Boolean.parseBoolean(System.getProperty("alice5.realtime", "false"));

      if (joystickCsg && !realtime)
      {
         System.out.println("[demo] -Dalice5.csg=joystick requires -Dalice5.realtime=true (no scripted gate loop in joystick mode)");
         System.exit(3);
      }

      Alice5RobotModel robotModel = new Alice5RobotModel(Alice5Version.V1_FULL_ROBOT, RobotTarget.SCS);
      FlatGroundEnvironment environment = new FlatGroundEnvironment();

      SCS2AvatarSimulationFactory factory = new SCS2AvatarSimulationFactory();
      factory.setRobotModel(robotModel);
      factory.setCommonAvatarEnvrionmentInterface(environment);
      factory.setCreateYoVariableServer(createYoVariableServer);
      factory.setInitializeEstimatorToActual(true);
      factory.setUseImpulseBasedPhysicsEngine(false);
      factory.setUseBulletPhysicsEngine(false);
      factory.setUsePerfectSensors(true);
      factory.setShowGUI(false);
      factory.setAutomaticallyStartSimulation(false);
      if (joystickCsg)
      {
         // SIM-EXT V1 teleop mode: JoystickBasedSteppingPluginFactory. The CSG inputs are the
         // walk_/desiredVelocity_/desiredTurningVelocity_StepGeneratorCommandInputManager
         // yo-variables (stepGenerator thread registry, published by the YoVariableServer and
         // writable through its variable-change channel).
         factory.setDefaultHighLevelHumanoidControllerFactory(false, null);
      }
      else
      {
         // Script mode: the heading-and-velocity script supplies the velocity profile (step-in-place 5s ->
         // cruise straight 6s -> turns/side-steps, repeating). In this mode the CSG walk input provider is
         // null, so the walkCSG yo-variable is externally writable (joystick mode clobbers it every tick).
         HeadingAndVelocityEvaluationScriptParameters scriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
         // Empirically (run dbg2/dbg3): the script velocity reaches the CSG as a UNIT velocity, scaled by
         // maxVelocityX = maxStepLength / (swing + transfer). Convert the commanded m/s into that fraction.
         double maxVelocityX = maxStepLength / (swingTime + transferTime);
         double velocityFraction = Math.min(1.0, velocity / maxVelocityX);
         scriptParameters.setCruiseVelocity(velocityFraction);
         scriptParameters.setMaxVelocity(velocityFraction);
         scriptParameters.setSideStepVelocity(Math.min(velocityFraction, 0.5));
         factory.setDefaultHighLevelHumanoidControllerFactory(true, scriptParameters);
      }

      double kxy = Double.parseDouble(System.getProperty("alice5.gc.kxy", "150000"));
      double bxy = Double.parseDouble(System.getProperty("alice5.gc.bxy", "1000"));
      factory.setGroundContactModelParameters(new GroundContactModelParameters(4000, 750, kxy, bxy));

      SCS2AvatarSimulation avatarSimulation = factory.createAvatarSimulation();
      avatarSimulation.setSystemExitOnDestroy(false);
      avatarSimulation.start();

      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
      FloatingJointBasics rootJoint = avatarSimulation.getControllerFullRobotModel().getRootJoint();
      YoRegistry root = scs.getRootRegistry();

      YoVariable walkingState = findBySuffix(root, "walkingCurrentState");
      YoBoolean walkCSG = (YoBoolean) findExact(root, "walkCSG");
      YoVariable desiredVelX = findExact(root, "desiredVelocityCSGX");
      YoVariable scriptEvent = findExact(root, "currentScriptEvent");
      YoDouble swingCSG = (YoDouble) findExact(root, "swingTimeCSG");
      YoDouble transferCSG = (YoDouble) findExact(root, "transferTimeCSG");
      YoDouble maxStepLengthCSG = (YoDouble) findExact(root, "maxStepLengthCSG");
      YoVariable tickTimer = findBySuffix(root, "controllerThreadTimerCurrent");
      YoVariable icpErrorX = findBySuffix(root, "ICPErrorX");
      YoVariable icpErrorY = findBySuffix(root, "ICPErrorY");

      List<String> missing = new ArrayList<>();
      if (walkingState == null) missing.add("walkingCurrentState");
      if (walkCSG == null && !joystickCsg) missing.add("walkCSG");
      if (tickTimer == null) missing.add("controllerThreadTimerCurrent");
      if (!missing.isEmpty())
      {
         System.out.println("GATE_M2_FAILURE missing yoVariables: " + missing);
         System.out.println("GATE_M2_FAIL");
         avatarSimulation.destroy();
         System.exit(3);
      }

      if (joystickCsg)
      {
         // Teleop mode: no pre-commanded walk. Report the CSG input variables that the remote
         // teleop client will write through the YoVariableServer change channel.
         YoVariable walkInput = findExact(root, "walk_StepGeneratorCommandInputManager");
         YoVariable velInputX = findExact(root, "desiredVelocity_StepGeneratorCommandInputManagerX");
         YoVariable velInputY = findExact(root, "desiredVelocity_StepGeneratorCommandInputManagerY");
         YoVariable turnInput = findExact(root, "desiredTurningVelocity_StepGeneratorCommandInputManager");
         System.out.println("[demo] csg=joystick walkInput=" + fullName(walkInput) + " velX=" + fullName(velInputX)
               + " velY=" + fullName(velInputY) + " turn=" + fullName(turnInput));
         if (walkInput == null || velInputX == null || velInputY == null || turnInput == null)
         {
            System.out.println("SIMEXT_RT_FAIL missing StepGeneratorCommandInputManager yoVariables");
            avatarSimulation.destroy();
            System.exit(3);
         }
      }
      else
      {
         // Command walking BEFORE the first simulation tick (writes from this thread during the run are
         // overwritten by the controller-side tasks; pre-run writes land -- same pattern as the upstream
         // open-alexander track). The script's STEP_IN_PLACE first event doubles as the settle phase.
         if (swingCSG != null)
            swingCSG.set(swingTime);
         if (transferCSG != null)
            transferCSG.set(transferTime);
         if (maxStepLengthCSG != null)
            maxStepLengthCSG.set(maxStepLength);
         walkCSG.set(true);
         System.out.println("[demo] walk pre-commanded (script velocity profile, cruise=" + velocity + ", swing=" + swingTime + ", transfer=" + transferTime + ")");
      }

      // SIM-EXT ARM-1: optional 1 Hz arm jointspace diagnostic (desired q_d_* vs actual q).
      boolean armDiag = Boolean.parseBoolean(System.getProperty("alice5.armDiag", "false"));
      String[] armDiagNames = {"l_sh_p", "l_sh_r", "l_el_p", "r_sh_p", "r_sh_r", "r_el_p"};
      us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[] armDiagJoints =
            new us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[armDiagNames.length];
      YoVariable[] armDiagDesired = new YoVariable[armDiagNames.length];
      if (armDiag)
      {
         for (int i = 0; i < armDiagNames.length; i++)
         {
            armDiagJoints[i] = avatarSimulation.getControllerFullRobotModel().getOneDoFJointByName(armDiagNames[i]);
            armDiagDesired[i] = findExact(root, "q_d_" + armDiagNames[i]);
         }
      }

      // step counting: listener on walking state transitions into single support
      final int[] stepCount = {0};
      walkingState.addListener(v -> {
         String s = v.getValueAsString();
         if (s.equals("WALKING_LEFT_SUPPORT") || s.equals("WALKING_RIGHT_SUPPORT"))
            stepCount[0]++;
      });

      // controller tick compute histogram (seconds)
      List<Double> tickSamples = new ArrayList<>(200000);
      tickTimer.addListener(v -> {
         double val = v.getValueAsDouble();
         if (val > 0)
            tickSamples.add(val);
      });

      if (realtime)
      {
         // SIM-EXT V1: real-time pacing branch. Never returns (exits the JVM). Push and qpos-CSV
         // options are not supported here; the M2 gate loop below is bypassed entirely.
         runRealtimePaced(avatarSimulation, scs, rootJoint, duration, stepCount, walkingState);
      }

      PushRobotControllerSCS2 pushController = null;
      if (pushTime > 0)
      {
         pushController = new PushRobotControllerSCS2(scs.getTime(), avatarSimulation.getRobot(), avatarSimulation.getControllerFullRobotModel());
      }

      // Optional 50 Hz qpos trajectory dump for offline MuJoCo video rendering.
      String qposCsvPath = System.getProperty("alice5.qpos.csv", "");
      PrintWriter qposCsv = null;
      String[] orderedJointNames = robotModel.getJointMap().getOrderedJointNames();
      us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[] recJoints =
            new us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[orderedJointNames.length];
      if (!qposCsvPath.isEmpty())
      {
         for (int i = 0; i < orderedJointNames.length; i++)
            recJoints[i] = avatarSimulation.getControllerFullRobotModel().getOneDoFJointByName(orderedJointNames[i]);
         try
         {
            qposCsv = new PrintWriter(qposCsvPath);
            StringBuilder header = new StringBuilder("t,px,py,pz,qw,qx,qy,qz");
            for (String name : orderedJointNames)
               header.append(',').append(name);
            qposCsv.println(header);
         }
         catch (Exception e)
         {
            System.out.println("[demo] qpos csv open failed: " + e);
         }
      }

      boolean pass = true;
      List<String> failures = new ArrayList<>();
      double icpRmsSum = 0.0;
      int icpRmsCount = 0;
      double walkStartX = Double.NaN;
      double walkStartT = Double.NaN;
      boolean pushed = false;
      int stepsBeforeWarmupEnd = 0;
      List<Double> xHistory = new ArrayList<>();
      List<Double> yHistory = new ArrayList<>();

      try (PrintWriter csv = new PrintWriter(csvPath))
      {
         csv.println("t,rootX,rootY,rootZ,steps,icpErrX,icpErrY,walkingState,scriptEvent");

         for (double t = 1.0; t <= duration + 1e-6; t += 1.0)
         {
            if (Math.abs(t - settleTime) < 0.5 && Double.isNaN(walkStartT))
            {
               walkStartX = rootJoint.getJointPose().getX();
               walkStartT = t;
               stepsBeforeWarmupEnd = stepCount[0];
            }

            if (pushTime > 0 && !pushed && t >= pushTime)
            {
               pushController.applyForce(new Vector3D(1.0, 0.0, 0.0), pushForce, pushDuration);
               pushed = true;
               System.out.println("[demo] push applied at t=" + t + " force=" + pushForce + "N dur=" + pushDuration + "s");
            }

            boolean ok = true;
            if (qposCsv != null)
            {
               for (int sub = 0; sub < 50 && ok; sub++)
               {
                  ok = scs.simulateNow(0.02);
                  StringBuilder row = new StringBuilder();
                  row.append(String.format(Locale.ROOT, "%.3f", t - 1.0 + 0.02 * (sub + 1)));
                  var pose = rootJoint.getJointPose();
                  row.append(String.format(Locale.ROOT, ",%.5f,%.5f,%.5f", pose.getX(), pose.getY(), pose.getZ()));
                  var q = pose.getOrientation();
                  row.append(String.format(Locale.ROOT, ",%.6f,%.6f,%.6f,%.6f", q.getS(), q.getX(), q.getY(), q.getZ()));
                  for (us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics joint : recJoints)
                     row.append(String.format(Locale.ROOT, ",%.5f", joint == null ? 0.0 : joint.getQ()));
                  qposCsv.println(row);
               }
            }
            else
            {
               ok = scs.simulateNow(1.0);
            }
            if (!ok)
            {
               failures.add("simulateNow returned false at t=" + t);
               pass = false;
               break;
            }

            if (armDiag)
            {
               StringBuilder sb = new StringBuilder(String.format(Locale.ROOT, "[armdiag] t=%.1f", t));
               for (int i = 0; i < armDiagNames.length; i++)
               {
                  double q = armDiagJoints[i] == null ? Double.NaN : armDiagJoints[i].getQ();
                  String qd = armDiagDesired[i] == null ? "n/a" : String.format(Locale.ROOT, "%.4f", armDiagDesired[i].getValueAsDouble());
                  sb.append(String.format(Locale.ROOT, " %s q=%.4f q_d=%s", armDiagNames[i], q, qd));
               }
               System.out.println(sb);
            }

            if (Boolean.parseBoolean(System.getProperty("alice5.debug", "false")) && t >= settleTime + 3 && t < settleTime + 5)
            {
               for (YoVariable v : root.collectSubtreeVariables())
               {
                  String n = v.getName();
                  if (n.equals("desiredVelocityCSGX") || n.equals("swingTimeCSG") || n.equals("transferTimeCSG")
                        || n.equals("maxStepLengthCSG") || n.equals("walkCSG") || n.equals("numberOfFixedFootstepsCSG"))
                     System.out.println("[debug] " + v.getFullNameString() + " = " + v.getValueAsString());
               }
            }

            double x = rootJoint.getJointPose().getX();
            double y = rootJoint.getJointPose().getY();
            double z = rootJoint.getJointPose().getZ();
            xHistory.add(x);
            yHistory.add(y);
            double ex = icpErrorX != null ? icpErrorX.getValueAsDouble() : Double.NaN;
            double ey = icpErrorY != null ? icpErrorY.getValueAsDouble() : Double.NaN;
            if (!Double.isNaN(ex) && t > walkStartT)
            {
               icpRmsSum += ex * ex + ey * ey;
               icpRmsCount++;
            }
            String state = walkingState.getValueAsString();
            String event = scriptEvent != null ? scriptEvent.getValueAsString() : "n/a";
            csv.printf(Locale.ROOT, "%.2f,%.5f,%.5f,%.5f,%d,%.5f,%.5f,%s,%s%n", t, x, y, z, stepCount[0], ex, ey, state, event);

            if (z < 0.5)
            {
               failures.add("FALL detected at t=" + t + " (root z=" + z + ")");
               pass = false;
               break;
            }
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         failures.add("exception: " + e);
         pass = false;
      }

      if (qposCsv != null)
         qposCsv.close();

      double distance = Double.isNaN(walkStartX) ? 0.0 : rootJoint.getJointPose().getX() - walkStartX;
      double walkDuration = Double.isNaN(walkStartT) ? 1.0 : duration - walkStartT;
      // speed = max over 4 s sliding windows of horizontal displacement norm (direction-agnostic;
      // the script includes turns/side-steps so net forward x is not a fair speed measure)
      double avgSpeed = 0.0;
      int window = 4;
      for (int i = 0; i + window < xHistory.size(); i++)
      {
         double d = Math.hypot(xHistory.get(i + window) - xHistory.get(i), yHistory.get(i + window) - yHistory.get(i));
         avgSpeed = Math.max(avgSpeed, d / window);
      }
      int steps = stepCount[0] - stepsBeforeWarmupEnd;
      double icpRms = icpRmsCount > 0 ? Math.sqrt(icpRmsSum / (2.0 * icpRmsCount)) : Double.NaN;

      // tick percentiles, skipping first 5 s worth of warmup samples
      double p50 = Double.NaN, p99 = Double.NaN, pMax = Double.NaN;
      int warmupSkip = (int) Math.min(tickSamples.size() * 0.1, 5.0 / robotModel.getControllerDT());
      if (tickSamples.size() > warmupSkip + 100)
      {
         double[] sorted = tickSamples.subList(warmupSkip, tickSamples.size()).stream().mapToDouble(Double::doubleValue).sorted().toArray();
         p50 = sorted[(int) (sorted.length * 0.50)];
         p99 = sorted[(int) (sorted.length * 0.99)];
         pMax = sorted[sorted.length - 1];
      }

      if (pass && steps < minSteps)
      {
         failures.add("steps " + steps + " < required " + minSteps);
         pass = false;
      }
      if (pass && minSpeed > 0 && avgSpeed < minSpeed)
      {
         failures.add("avg speed " + avgSpeed + " < required " + minSpeed);
         pass = false;
      }
      if (pushTime > 0 && !pushed)
      {
         failures.add("push requested but never applied");
         pass = false;
      }

      System.out.println(String.format(Locale.ROOT,
            "GATE_M2_METRICS steps=%d distance=%.3f avgSpeed=%.4f icpRms=%.4f tickP50=%.5f tickP99=%.5f tickMax=%.5f nTicks=%d pushed=%b",
            steps, distance, avgSpeed, icpRms, p50, p99, pMax, tickSamples.size(), pushed));
      for (String failure : failures)
         System.out.println("GATE_M2_FAILURE " + failure);
      System.out.println(pass ? "GATE_M2_PASS" : "GATE_M2_FAIL");

      avatarSimulation.destroy();
      System.exit(pass ? 0 : 2);
   }

   /**
    * SIM-EXT V1: run the already-started simulation at 1.0x wall clock and monitor it from the main
    * thread (no yoVariable writes from here -- mid-run main-thread writes are lost, see M2 notes).
    * Exits the JVM: 0 = completed, 2 = fall or early stop.
    */
   static void runRealtimePaced(SCS2AvatarSimulation avatarSimulation, SimulationConstructionSet2 scs, FloatingJointBasics rootJoint,
                                double duration, int[] stepCount, YoVariable walkingState)
   {
      scs.setRealTimeRateSimulation(true);
      scs.simulate(duration);
      System.out.println("[rt] real-time pacing enabled (1.0x wall clock), duration=" + duration + " s");

      boolean fall = false;
      boolean stoppedEarly = false;
      double lastT = -1.0;
      int stallTicks = 0;
      while (true)
      {
         try
         {
            Thread.sleep(1000);
         }
         catch (InterruptedException e)
         {
            break;
         }
         double t = scs.getTime().getValue();
         double z = rootJoint.getJointPose().getZ();
         System.out.println(String.format(Locale.ROOT, "[rt] t=%.1f z=%.3f steps=%d state=%s",
                                          t, z, stepCount[0], walkingState.getValueAsString()));
         if (z < 0.5)
         {
            System.out.println("[rt] FALL detected (root z=" + z + ")");
            fall = true;
            break;
         }
         if (t >= duration - 1e-3)
            break;
         if (t == lastT)
         {
            stallTicks++;
            if (stallTicks >= 10 && !scs.isSimulating())
            {
               System.out.println("[rt] simulation stopped early at t=" + t);
               stoppedEarly = true;
               break;
            }
         }
         else
         {
            stallTicks = 0;
         }
         lastT = t;
      }

      System.out.println(fall || stoppedEarly ? "SIMEXT_RT_FAIL" : "SIMEXT_RT_DONE");
      avatarSimulation.destroy();
      System.exit(fall || stoppedEarly ? 2 : 0);
   }

   static String fullName(YoVariable variable)
   {
      return variable == null ? "MISSING" : variable.getFullNameString();
   }

   static YoVariable findBySuffix(YoRegistry registry, String suffix)
   {
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         if (variable.getName().endsWith(suffix))
            return variable;
      }
      return null;
   }

   static YoVariable findExact(YoRegistry registry, String name)
   {
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         if (variable.getName().equals(name))
            return variable;
      }
      return null;
   }
}
