package us.ihmc.alice5;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
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

      // M5 conservative forward profile: a fixed FootstepDataListMessage (4 in-place steps then forward
      // steps at the profile step length) is submitted to the walking controller's CommandInputManager.
      // NO eval-script (the shared HeadingAndVelocityEvaluationScript turns 180 at t~12) and NO continuous
      // step generator -- the standing-balance walking state consumes the queue directly. Step geometry,
      // swing/transfer timing, and swing-arc height are commanded explicitly per the conservative profile.
      boolean m5Conservative = "m5conservative".equalsIgnoreCase(System.getProperty("alice5.scenario", ""));

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
      HighLevelHumanoidControllerFactory m5ControllerFactory = null;
      if (m5Conservative)
      {
         // M5 conservative forward profile: forward motion is commanded by submitting an explicit
         // FootstepDataListMessage to the walking controller's CommandInputManager (see the gate loop):
         // the controller's standing-balance "WALKING" high-level state consumes a footstep queue
         // directly, so there is no eval-script (no turns/side-steps at t~12) and no continuous-step-
         // generator walk-trigger to coax. Geometry is commanded directly: 4 in-place steps then forward
         // steps at the profile step length. The script=false+null path installs the joystick stepping
         // plugin whose generator stays idle (no walk input written), so it never auto-submits competing
         // footsteps -- our queue is the only footstep source. Capture the controller factory to reach
         // its CommandInputManager.
         m5ControllerFactory = factory.setDefaultHighLevelHumanoidControllerFactory(false, null);
      }
      else if (joystickCsg)
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

      // M5 realized forward-speed ceiling: the walking controller advances at most one maxStepLength step
      // every (swing+transfer) s, so steady forward speed = maxStepLength/(swing+transfer). The conservative
      // profile's written vel (0.1 m/s) exceeds this geometric ceiling (~0.067) -- documented in M5.md.
      double m5VelCeiling = maxStepLength / (swingTime + transferTime);

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

      // M5: build the conservative footstep queue and the controller command-input handle. The queue is
      // built lazily in the gate loop (after the controller has settled into standing balance) from the
      // live sole-frame world poses, then submitted once. See buildM5FootstepList.
      CommandInputManager m5CommandInput = null;
      boolean m5Submitted = false;
      int m5InPlaceSteps = Integer.parseInt(System.getProperty("alice5.inPlaceSteps", "4"));
      double m5SubmitTime = Double.parseDouble(System.getProperty("alice5.m5SubmitTime", "2.0"));
      double swingHeightProp = Double.parseDouble(System.getProperty("alice5.swingHeight", "0.04"));
      if (m5Conservative)
      {
         m5CommandInput = m5ControllerFactory != null ? m5ControllerFactory.getCommandInputManager() : null;
         if (m5CommandInput == null)
         {
            System.out.println("GATE_M2_FAILURE M5 controller CommandInputManager unavailable");
            System.out.println("GATE_M2_FAIL");
            avatarSimulation.destroy();
            System.exit(3);
         }
         System.out.println("[demo] M5 conservative profile: " + m5InPlaceSteps + " in-place steps -> forward at "
               + maxStepLength + " m/step (swing=" + swingTime + ", transfer=" + transferTime + ", swingHeight="
               + swingHeightProp + "); realized fwd speed ~= maxStep/(swing+transfer) = "
               + String.format(Locale.ROOT, "%.4f", m5VelCeiling) + " m/s (vel=" + velocity
               + " m/s as written exceeds this geometric ceiling -- see docs/gates/M5.md).");
      }

      if (m5Conservative)
      {
         // nothing pre-run; the footstep message is submitted in the gate loop after settling.
      }
      else if (joystickCsg)
      {
         // Teleop mode: no pre-commanded walk (remote client drives the CSG inputs at runtime).
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

      // SIM-EXT T2: optional foot sole-frame world-z dump for offline touchdown-impact analysis.
      // Both feet's sole-frame (contact surface) z are written each substep alongside the walking-state
      // string; vz is finite-differenced offline and the touchdown impact speed is the swing foot's |vz|
      // at ground contact (see scripts/twin/foot_touchdown.py). Off by default -> M2 path untouched.
      // -Dalice5.footdump.dt sets the sub-step / sample period (default 0.005 s = 200 Hz; 20 ms phase-
      // quantizes the contact velocity since the foot moves up to ~10 mm per 20 ms interval).
      String footCsvPath = System.getProperty("alice5.footdump.csv", "");
      double footDumpDt = Double.parseDouble(System.getProperty("alice5.footdump.dt", "0.005"));
      PrintWriter footCsv = null;
      us.ihmc.robotics.robotSide.SideDependentList<us.ihmc.mecano.frames.MovingReferenceFrame> soleFrames = null;
      // SIM-EXT FT: per-foot foot-switch contact boolean (wrench or kinematic, whichever is active),
      // logged next to the physical sole-frame z so the gate can score detection lead/lag offline.
      YoVariable lFootContact = null;
      YoVariable rFootContact = null;
      YoVariable lFootForceZ = null;
      YoVariable rFootForceZ = null;
      if (!footCsvPath.isEmpty())
      {
         soleFrames = avatarSimulation.getControllerFullRobotModel().getSoleFrames();
         lFootContact = findFootContactBoolean(root, "left");
         rFootContact = findFootContactBoolean(root, "right");
         // SIM-EXT FT: sole-frame vertical ground reaction (loadcell-derived) from the wrench switch.
         // Present only on the wrench path; on the kinematic path these stay null and log as NaN.
         lFootForceZ = findFootForceZ(root, "left");
         rFootForceZ = findFootForceZ(root, "right");
         System.out.println("[demo] foot contact vars: left=" + fullName(lFootContact) + " right=" + fullName(rFootContact));
         System.out.println("[demo] foot forceZ vars: left=" + fullName(lFootForceZ) + " right=" + fullName(rFootForceZ));
         try
         {
            footCsv = new PrintWriter(footCsvPath);
            footCsv.println("t,lSoleZ,rSoleZ,lContact,rContact,lForceZ,rForceZ,walkingState");
         }
         catch (Exception e)
         {
            System.out.println("[demo] foot csv open failed: " + e);
         }
      }

      // SIM-EXT VIZ-JOINT: optional joint q/qd/tau dump for offline render overlay (VIZ-JOINT).
      // Core walking joints (both legs hip_p/knee_p/ankle_p + hip_r). For each: q [rad] = getQ(),
      // qd [rad/s] = getQd() (true instantaneous, unlike the T3 finite-diff lower bound), and tau
      // [Nm] = getTau() on the controller fullRobotModel (the inverse-dynamics joint torque the WBC
      // solves). A desired-torque YoVariable (tau_d_<name>) is also looked up when present so the
      // gate can compare the two tau sources; logged as NaN when absent. Off by default -> the M2/T3
      // gate path is untouched (opt-in like the qpos/foot dumps). One-shot enumeration of every
      // YoVariable whose name contains "knee_p" is printed at open so the real desired-torque
      // variable name is discovered empirically rather than guessed.
      String jointCsvPath = System.getProperty("alice5.jointdump.csv", "");
      PrintWriter jointCsv = null;
      String[] vizJointNames = {"l_hip_p", "l_knee_p", "l_ankle_p", "l_ankle_r", "r_hip_p", "r_knee_p", "r_ankle_p", "r_ankle_r", "l_hip_r", "r_hip_r"};
      us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[] vizJoints =
            new us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics[vizJointNames.length];
      YoVariable[] vizTauDesired = new YoVariable[vizJointNames.length];
      if (!jointCsvPath.isEmpty())
      {
         for (int i = 0; i < vizJointNames.length; i++)
         {
            vizJoints[i] = avatarSimulation.getControllerFullRobotModel().getOneDoFJointByName(vizJointNames[i]);
            // Desired joint torque the WBC commands, if exposed as a YoVariable (analogous to q_d_).
            YoVariable tauD = findExact(root, "tau_d_" + vizJointNames[i]);
            if (tauD == null)
               tauD = findExact(root, "tau_" + vizJointNames[i]);
            vizTauDesired[i] = tauD;
         }
         // One-shot: enumerate every YoVariable mentioning a knee_p joint so the desired-torque
         // variable name (if any) is discovered from this single run instead of guessed.
         System.out.println("[demo] VIZ-JOINT knee_p yoVariable enumeration:");
         for (YoVariable v : root.collectSubtreeVariables())
            if (v.getName().contains("knee_p"))
               System.out.println("[viz-enum] " + v.getFullNameString());
         System.out.println("[demo] VIZ-JOINT tau_d vars: " + Arrays.toString(vizTauDesired));
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
            if (m5Conservative && !m5Submitted && t >= m5SubmitTime)
            {
               // Build the conservative footstep queue from the live sole poses now that the robot is in
               // standing balance, then submit it once. The walking controller's standing-balance state
               // consumes the FootstepDataListMessage and walks it out: in-place steps first (each foot
               // lifts and lands at its current spot), then forward steps at maxStepLength spacing.
               FootstepDataListMessage footMessage = buildM5FootstepList(avatarSimulation, swingTime, transferTime,
                     swingHeightProp, m5InPlaceSteps, maxStepLength, duration);
               m5CommandInput.submitMessage(footMessage);
               m5Submitted = true;
               System.out.println("[demo] M5 submitted footstep queue: " + footMessage.getFootstepDataList().size()
                     + " steps (" + m5InPlaceSteps + " in-place + forward) at t=" + t);
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
            if (qposCsv != null || footCsv != null || jointCsv != null)
            {
               // Sub-step the 1 s tick. Foot dump (touchdown vz) needs fine sampling (default 5 ms);
               // qpos video alone is fine at 20 ms. When the foot dump is on it sets the cadence.
               double subDt = footCsv != null ? footDumpDt : 0.02;
               int nSub = (int) Math.round(1.0 / subDt);
               for (int sub = 0; sub < nSub && ok; sub++)
               {
                  ok = scs.simulateNow(subDt);
                  double subT = t - 1.0 + subDt * (sub + 1);
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
                  if (footCsv != null)
                  {
                     us.ihmc.euclid.referenceFrame.FramePoint3D lSole =
                           new us.ihmc.euclid.referenceFrame.FramePoint3D(soleFrames.get(us.ihmc.robotics.robotSide.RobotSide.LEFT));
                     us.ihmc.euclid.referenceFrame.FramePoint3D rSole =
                           new us.ihmc.euclid.referenceFrame.FramePoint3D(soleFrames.get(us.ihmc.robotics.robotSide.RobotSide.RIGHT));
                     lSole.changeFrame(us.ihmc.euclid.referenceFrame.ReferenceFrame.getWorldFrame());
                     rSole.changeFrame(us.ihmc.euclid.referenceFrame.ReferenceFrame.getWorldFrame());
                     int lC = lFootContact != null && lFootContact.getValueAsDouble() != 0.0 ? 1 : 0;
                     int rC = rFootContact != null && rFootContact.getValueAsDouble() != 0.0 ? 1 : 0;
                     double lFz = lFootForceZ != null ? lFootForceZ.getValueAsDouble() : Double.NaN;
                     double rFz = rFootForceZ != null ? rFootForceZ.getValueAsDouble() : Double.NaN;
                     footCsv.println(String.format(Locale.ROOT, "%.3f,%.6f,%.6f,%d,%d,%.3f,%.3f,%s",
                           subT, lSole.getZ(), rSole.getZ(), lC, rC, lFz, rFz, walkingState.getValueAsString()));
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
      if (footCsv != null)
         footCsv.close();
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
            "GATE_M2_METRICS steps=%d distance=%.3f avgSpeed=%.4f icpRms=%.4f tickP50=%.5f tickP99=%.5f tickMax=%.5f nTicks=%d pushed=%b",
            steps, distance, avgSpeed, icpRms, p50, p99, pMax, tickSamples.size(), pushed));
      for (String failure : failures)
         System.out.println("GATE_M2_FAILURE " + failure);
      System.out.println(pass ? "GATE_M2_PASS" : "GATE_M2_FAIL");

      avatarSimulation.destroy();
      System.exit(pass ? 0 : 2);
   }

   /**
    * M5 conservative footstep queue: {@code inPlaceSteps} alternating in-place steps (each foot lifts and
    * lands at its current sole position) followed by forward steps advancing x by {@code stepLength} each,
    * until the queue fills the remaining run time. Step timing is the conservative profile (swing/transfer)
    * and swing arc height is the T2 value, set both as the list defaults and per-step. World-frame poses
    * are taken from the live sole frames so the queue is anchored to the robot's actual stance.
    */
   static FootstepDataListMessage buildM5FootstepList(SCS2AvatarSimulation avatarSimulation, double swingTime,
                                                      double transferTime, double swingHeight, int inPlaceSteps,
                                                      double stepLength, double duration)
   {
      SideDependentList<MovingReferenceFrame> soleFrames = avatarSimulation.getControllerFullRobotModel().getSoleFrames();
      FramePoint3D leftSole = new FramePoint3D(soleFrames.get(RobotSide.LEFT));
      FramePoint3D rightSole = new FramePoint3D(soleFrames.get(RobotSide.RIGHT));
      leftSole.changeFrame(ReferenceFrame.getWorldFrame());
      rightSole.changeFrame(ReferenceFrame.getWorldFrame());
      SideDependentList<FramePoint3D> startSole = new SideDependentList<>(leftSole, rightSole);

      FootstepDataListMessage message = new FootstepDataListMessage();
      message.setDefaultSwingDuration(swingTime);
      message.setDefaultTransferDuration(transferTime);
      message.setFinalTransferDuration(transferTime);

      // Total step budget: keep the queue within the run window. Each step costs (swing+transfer) s, leave
      // a settle margin. The forward steps fill whatever remains after the in-place phase.
      double stepTime = swingTime + transferTime;
      int totalSteps = Math.max(inPlaceSteps + 2, (int) ((duration - 4.0) / stepTime));
      int forwardSteps = Math.max(2, totalSteps - inPlaceSteps);

      RobotSide side = RobotSide.LEFT;
      // In-place phase: alternate feet, each landing back at its own start sole position.
      for (int i = 0; i < inPlaceSteps; i++)
      {
         FramePoint3D p = startSole.get(side);
         addM5Footstep(message, side, p.getX(), p.getY(), swingHeight);
         side = side.getOppositeSide();
      }
      // Forward phase: advance the stepping foot's x by stepLength relative to the previous same-side x.
      // Increment x by stepLength on each step so the feet march forward together at stepLength spacing.
      double advance = 0.0;
      for (int i = 0; i < forwardSteps; i++)
      {
         advance += stepLength;
         FramePoint3D p = startSole.get(side);
         addM5Footstep(message, side, p.getX() + advance, p.getY(), swingHeight);
         side = side.getOppositeSide();
      }
      return message;
   }

   static void addM5Footstep(FootstepDataListMessage message, RobotSide side, double x, double y, double swingHeight)
   {
      FootstepDataMessage footstep = message.getFootstepDataList().add();
      footstep.setRobotSide(side.toByte());
      footstep.getLocation().set(x, y, 0.0);
      footstep.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstep.setSwingHeight(swingHeight);
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

   /**
    * SIM-EXT FT: per-foot contact boolean of the active foot switch, found by name. The wrench switch
    * names it {@code <footName>FootHitGroundFiltered}, the kinematic switch {@code <footName>hitGround};
    * the foot name carries the side token (left_ankle_roll / right_ankle_roll). Returns the filtered
    * contact variable the gait state machine consumes, for either switch type, or null if absent.
    */
   static YoVariable findFootContactBoolean(YoRegistry registry, String sideToken)
   {
      YoVariable wrench = null;
      YoVariable kinematic = null;
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         String name = variable.getName();
         if (!name.contains(sideToken))
            continue;
         if (name.endsWith("FootHitGroundFiltered"))
            wrench = variable;
         else if (name.endsWith("hitGround") && !name.endsWith("FootHitGround"))
            kinematic = variable;
      }
      return wrench != null ? wrench : kinematic;
   }

   /**
    * SIM-EXT FT: sole-frame vertical ground reaction component {@code <footName>ForceSoleFrameZ} of the
    * wrench foot switch (the loadcell-derived load injected through the bridge). Used as the
    * control-relevant contact truth (load bearing) for touchdown/liftoff scoring. Null on the
    * kinematic path (that switch publishes no force vector).
    */
   static YoVariable findFootForceZ(YoRegistry registry, String sideToken)
   {
      // Target the wrench foot switch's own measured sole-frame reaction (namePrefix +
      // "ForceSoleFrameZ"), not the QP WrenchVisualizer's DesiredExternalForceSoleFrameZ: require the
      // full path to be under a WrenchBasedFootSwitch registry and the leaf to be exactly the foot
      // prefix + ForceSoleFrameZ (no "Desired" / "External").
      for (YoVariable variable : registry.collectSubtreeVariables())
      {
         String name = variable.getName();
         if (!name.endsWith("ForceSoleFrameZ") || name.contains("Desired") || name.contains("External"))
            continue;
         if (!name.contains(sideToken))
            continue;
         if (variable.getFullNameString().contains("WrenchBasedFootSwitch"))
            return variable;
      }
      return null;
   }
}
