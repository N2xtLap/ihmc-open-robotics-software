package us.ihmc.alice5;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
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
 */
public class Alice5TerrainWalkingDemo
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

      // SIM-EXT T1: -Dalice5.terrain=random|ramp|steps switches to uneven terrain + direct CSG
      // command injection (the heading/velocity script turns after 6 s of cruise, which walks off
      // ramps; terrain runs need sustained straight walking instead).
      String terrainType = System.getProperty("alice5.terrain", "flat");
      boolean terrainMode = !terrainType.equals("flat");
      double swingHeight = Double.parseDouble(System.getProperty("alice5.swingHeight", "0.10"));

      Alice5RobotModel robotModel = new Alice5RobotModel(Alice5Version.V1_FULL_ROBOT, RobotTarget.SCS);
      CommonAvatarEnvironmentInterface environment = Alice5TerrainEnvironment.fromSystemProperties();
      Alice5TerrainEnvironment terrainEnvironment = terrainMode ? (Alice5TerrainEnvironment) environment : null;

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
      // Script mode: the heading-and-velocity script supplies the velocity profile (step-in-place 5s ->
      // cruise straight 6s -> turns/side-steps, repeating). In this mode the CSG walk input provider is
      // null, so the walkCSG yo-variable is externally writable (joystick mode clobbers it every tick).
      if (terrainMode)
      {
         // Joystick plugin mode: CSG providers read from StepGeneratorCommandInputManager, which we
         // feed via its thread-safe CommandInputManager message queue from the main loop (yoVariable
         // writes from this thread are lost mid-run; the message queue is the supported cross-thread
         // path). No height map is passed, so footstep z stays blind on the uneven terrain.
         factory.setDefaultHighLevelHumanoidControllerFactory(false, null);
      }
      else
      {
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
      double kz = Double.parseDouble(System.getProperty("alice5.gc.kz", "4000"));
      double bz = Double.parseDouble(System.getProperty("alice5.gc.bz", "750"));
      // Per-point spring parameters: when -Dalice5.simContactGrid multiplies the contact point
      // count, scale these down by (4 / nPoints) to keep the effective sole stiffness (M1: 5x
      // lateral stiffness diverges).
      factory.setGroundContactModelParameters(new GroundContactModelParameters(kz, bz, kxy, bxy));

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
      if (walkCSG == null) missing.add("walkCSG");
      if (tickTimer == null) missing.add("controllerThreadTimerCurrent");
      if (!missing.isEmpty())
      {
         System.out.println("GATE_T1_FAILURE missing yoVariables: " + missing);
         System.out.println("GATE_T1_FAIL");
         avatarSimulation.destroy();
         System.exit(3);
      }

      // Command walking BEFORE the first simulation tick (writes from this thread during the run are
      // overwritten by the controller-side tasks; pre-run writes land — same pattern as the upstream
      // open-alexander track). The script's STEP_IN_PLACE first event doubles as the settle phase.
      StepGeneratorCommandInputManager csgInput = null;
      if (terrainMode)
      {
         csgInput = avatarSimulation.getStepGeneratorThread().getCsgCommandInputManager();
         // All CSG parameters via pre-first-tick yoVariable writes (the only reliable window, M2
         // finding). A ContinuousStepGeneratorParametersMessage would clobber stepsAreAdjustable
         // back to false on every submit (the message has no field for it), so it is not used.
         setYoDouble(root, "swingTimeCSG", swingTime);
         setYoDouble(root, "transferTimeCSG", transferTime);
         setYoDouble(root, "maxStepLengthCSG", maxStepLength);
         setYoDouble(root, "swingHeightCSG", swingHeight);
         String stepWidthProp = System.getProperty("alice5.stepWidth", "");
         if (!stepWidthProp.isEmpty())
            setYoDouble(root, "inPlaceWidthCSG", Double.parseDouble(stepWidthProp));
         if (Boolean.parseBoolean(System.getProperty("alice5.shiftTouchdown", "false")))
            setYoBoolean(root, "shiftUpcomingStepsWithTouchdownCSG", true);
         if (Boolean.parseBoolean(System.getProperty("alice5.adjustableSteps", "false")))
            setYoBoolean(root, "stepsAreAdjustableCSG", true);
         System.out.println("[demo] terrain mode: " + terrainType + " (CSG commands via message queue, vel=" + velocity
               + " m/s, swing=" + swingTime + ", transfer=" + transferTime + ", swingHeight=" + swingHeight + ")");
      }
      else
      {
         if (swingCSG != null)
            swingCSG.set(swingTime);
         if (transferCSG != null)
            transferCSG.set(transferTime);
         if (maxStepLengthCSG != null)
            maxStepLengthCSG.set(maxStepLength);
         walkCSG.set(true);
         System.out.println("[demo] walk pre-commanded (script velocity profile, cruise=" + velocity + ", swing=" + swingTime + ", transfer=" + transferTime + ")");
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

      // SIM-EXT T4: optional joint q/qd/tau dump for offline render overlay (VIZ-JOINT format).
      // Mirrors Alice5WalkingDemo's VIZ-JOINT dump exactly so render_qpos_joints.py reads it
      // unchanged. Core sagittal leg joints + hip_r: q [rad]=getQ(), qd [rad/s]=getQd(),
      // tau [Nm]=getTau() on the controller fullRobotModel, and a desired-torque YoVariable
      // (tau_d_/tau_) logged as NaN when absent. Off by default -> the T1 gate path is untouched
      // (opt-in like the qpos dump); the substep loop only engages when a dump CSV is requested.
      double dumpSubDt = Double.parseDouble(System.getProperty("alice5.dumpSubDt", "0.02"));
      int dumpNSub = (int) Math.round(1.0 / dumpSubDt);
      String jointCsvPath = System.getProperty("alice5.jointdump.csv", "");
      PrintWriter jointCsv = null;
      String[] vizJointNames = {"l_hip_p", "l_knee_p", "l_ankle_p", "r_hip_p", "r_knee_p", "r_ankle_p", "l_hip_r", "r_hip_r"};
      us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[] vizJoints =
            new us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[vizJointNames.length];
      YoVariable[] vizTauDesired = new YoVariable[vizJointNames.length];
      if (!jointCsvPath.isEmpty())
      {
         for (int i = 0; i < vizJointNames.length; i++)
         {
            vizJoints[i] = avatarSimulation.getControllerFullRobotModel().getOneDoFJointByName(vizJointNames[i]);
            YoVariable tauD = findExact(root, "tau_d_" + vizJointNames[i]);
            if (tauD == null)
               tauD = findExact(root, "tau_" + vizJointNames[i]);
            vizTauDesired[i] = tauD;
         }
         try
         {
            jointCsv = new PrintWriter(jointCsvPath);
            StringBuilder header = new StringBuilder("t");
            for (String name : vizJointNames)
               header.append(",q_").append(name).append(",qd_").append(name)
                     .append(",tau_").append(name).append(",taud_").append(name);
            header.append(",walkingState");
            jointCsv.println(header);
         }
         catch (Exception e)
         {
            System.out.println("[demo] joint csv open failed: " + e);
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
            if (csgInput != null)
            {
               // Resubmit every loop second: the input manager drops commands until the high-level
               // WALKING status opens it, and resubmission is idempotent afterwards.
               ContinuousStepGeneratorInputMessage inputMessage = new ContinuousStepGeneratorInputMessage();
               inputMessage.setWalk(t >= settleTime);
               inputMessage.setForwardVelocity(velocity);
               inputMessage.setLateralVelocity(0.0);
               inputMessage.setTurnVelocity(0.0);
               // CSG semantics: isUnitVelocity()==true means the value is in physical units (m/s), clamped to
               // maxStepLength/stepTime; false means a unit FRACTION of that max (script-mode behavior).
               inputMessage.setUnitVelocities(true);
               csgInput.getCommandInputManager().submitMessage(inputMessage);
            }

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
            if (qposCsv != null || jointCsv != null)
            {
               for (int sub = 0; sub < dumpNSub && ok; sub++)
               {
                  ok = scs.simulateNow(dumpSubDt);
                  double subT = t - 1.0 + dumpSubDt * (sub + 1);
                  if (qposCsv != null)
                  {
                     StringBuilder row = new StringBuilder();
                     row.append(String.format(Locale.ROOT, "%.3f", subT));
                     var pose = rootJoint.getJointPose();
                     row.append(String.format(Locale.ROOT, ",%.5f,%.5f,%.5f", pose.getX(), pose.getY(), pose.getZ()));
                     var q = pose.getOrientation();
                     row.append(String.format(Locale.ROOT, ",%.6f,%.6f,%.6f,%.6f", q.getS(), q.getX(), q.getY(), q.getZ()));
                     for (us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics joint : recJoints)
                        row.append(String.format(Locale.ROOT, ",%.5f", joint == null ? 0.0 : joint.getQ()));
                     qposCsv.println(row);
                  }
                  if (jointCsv != null)
                  {
                     StringBuilder row = new StringBuilder();
                     row.append(String.format(Locale.ROOT, "%.3f", subT));
                     for (int i = 0; i < vizJoints.length; i++)
                     {
                        us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics j = vizJoints[i];
                        double q = j == null ? Double.NaN : j.getQ();
                        double qd = j == null ? Double.NaN : j.getQd();
                        double tau = j == null ? Double.NaN : j.getTau();
                        double taud = vizTauDesired[i] == null ? Double.NaN : vizTauDesired[i].getValueAsDouble();
                        row.append(String.format(Locale.ROOT, ",%.5f,%.5f,%.4f,%.4f", q, qd, tau, taud));
                     }
                     row.append(',').append(walkingState.getValueAsString());
                     jointCsv.println(row);
                  }
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

            if (Boolean.parseBoolean(System.getProperty("alice5.debug", "false")) && t >= settleTime + 3 && t < settleTime + 5)
            {
               for (YoVariable v : root.collectSubtreeVariables())
               {
                  String n = v.getName();
                  if (n.equals("desiredVelocityCSGX") || n.equals("swingTimeCSG") || n.equals("transferTimeCSG")
                        || n.equals("maxStepLengthCSG") || n.equals("walkCSG") || n.equals("isUnitVelocities") || n.equals("numberOfFixedFootstepsCSG"))
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

            double groundZ = terrainMode ? terrainEnvironment.heightAt(x, y) : 0.0;
            if (z - groundZ < 0.5)
            {
               failures.add("FALL detected at t=" + t + " (root z=" + z + ", ground z=" + groundZ + ")");
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
      if (jointCsv != null)
         jointCsv.close();

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
            "GATE_T1_METRICS steps=%d distance=%.3f avgSpeed=%.4f icpRms=%.4f tickP50=%.5f tickP99=%.5f tickMax=%.5f nTicks=%d pushed=%b",
            steps, distance, avgSpeed, icpRms, p50, p99, pMax, tickSamples.size(), pushed));
      for (String failure : failures)
         System.out.println("GATE_T1_FAILURE " + failure);
      System.out.println(pass ? "GATE_T1_PASS" : "GATE_T1_FAIL");

      avatarSimulation.destroy();
      System.exit(pass ? 0 : 2);
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

   static void setYoDouble(YoRegistry registry, String name, double value)
   {
      YoVariable variable = findExact(registry, name);
      if (variable instanceof YoDouble)
      {
         ((YoDouble) variable).set(value);
         System.out.println("[demo] pre-run " + name + "=" + value);
      }
      else
         System.out.println("[demo] WARN yoVariable not found: " + name);
   }

   static void setYoBoolean(YoRegistry registry, String name, boolean value)
   {
      YoVariable variable = findExact(registry, name);
      if (variable instanceof YoBoolean)
      {
         ((YoBoolean) variable).set(value);
         System.out.println("[demo] pre-run " + name + "=" + value);
      }
      else
         System.out.println("[demo] WARN yoVariable not found: " + name);
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
