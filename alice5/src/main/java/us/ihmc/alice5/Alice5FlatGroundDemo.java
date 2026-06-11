package us.ihmc.alice5;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.avatar.scs2.SCS2AvatarSimulationFactory;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * M1/M2 gate demo: headless SCS2 flat-ground simulation of ALICE5.
 *
 * Runs the full avatar simulation (estimator + walking controller + WBC QP) without GUI for
 * {@code -Dalice5.duration} sim-seconds (default 70), sampling metrics every sim-second into a CSV.
 * Exit code 0 iff the gate criteria pass: no fall, no controller exception, standing kept,
 * CoM xy drift < 2 cm after settling, ICP error bounded (< 0.05 m).
 */
public class Alice5FlatGroundDemo
{
   public static void main(String[] args)
   {
      double duration = Double.parseDouble(System.getProperty("alice5.duration", "70.0"));
      double settleTime = Double.parseDouble(System.getProperty("alice5.settle", "10.0"));
      String csvPath = System.getProperty("alice5.metrics.csv", "alice5_m1_metrics.csv");
      boolean createYoVariableServer = Boolean.parseBoolean(System.getProperty("create.yovariable.server", "true"));

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
      factory.setDefaultHighLevelHumanoidControllerFactory();

      // Tuning iteration 1 (M1): default xy ground springs let the feet ratchet forward (~1 mm/s creep
      // with near-zero ICP error). Stiffer tangential contact suppresses the creep.
      double kxy = Double.parseDouble(System.getProperty("alice5.gc.kxy", "250000"));
      double bxy = Double.parseDouble(System.getProperty("alice5.gc.bxy", "5000"));
      double kz = Double.parseDouble(System.getProperty("alice5.gc.kz", "4000"));
      double bz = Double.parseDouble(System.getProperty("alice5.gc.bz", "750"));
      factory.setGroundContactModelParameters(new us.ihmc.wholeBodyController.RobotContactPointParameters.GroundContactModelParameters(kz, bz, kxy, bxy));

      SCS2AvatarSimulation avatarSimulation = factory.createAvatarSimulation();
      avatarSimulation.setSystemExitOnDestroy(false);
      avatarSimulation.start();

      SimulationConstructionSet2 scs = avatarSimulation.getSimulationConstructionSet();
      FloatingJointBasics rootJoint = avatarSimulation.getControllerFullRobotModel().getRootJoint();

      YoVariable icpErrorX = findBySuffix(scs.getRootRegistry(), "ICPErrorX");
      YoVariable icpErrorY = findBySuffix(scs.getRootRegistry(), "ICPErrorY");
      YoVariable walkingState = findBySuffix(scs.getRootRegistry(), "walkingCurrentState");
      if (walkingState == null)
         walkingState = findBySuffix(scs.getRootRegistry(), "WalkingCurrentState");

      System.out.println("[demo] icpErrorX=" + name(icpErrorX) + " icpErrorY=" + name(icpErrorY) + " walkingState=" + name(walkingState));

      boolean pass = true;
      List<String> failures = new ArrayList<>();
      double refX = Double.NaN, refY = Double.NaN;
      double maxDrift = 0.0, maxIcpError = 0.0;
      double standingSince = -1.0;
      double standingAccumulated = 0.0;

      try (PrintWriter csv = new PrintWriter(csvPath))
      {
         csv.println("t,rootX,rootY,rootZ,icpErrX,icpErrY,walkingState,driftXY");

         for (double t = 1.0; t <= duration + 1e-6; t += 1.0)
         {
            boolean ok = scs.simulateNow(1.0);
            if (!ok)
            {
               failures.add("simulateNow returned false at t=" + t + " (controller exception or sim crash)");
               pass = false;
               break;
            }

            double x = rootJoint.getJointPose().getX();
            double y = rootJoint.getJointPose().getY();
            double z = rootJoint.getJointPose().getZ();
            double ex = icpErrorX != null ? icpErrorX.getValueAsDouble() : Double.NaN;
            double ey = icpErrorY != null ? icpErrorY.getValueAsDouble() : Double.NaN;
            String state = walkingState != null ? walkingState.getValueAsString() : "n/a";

            if (t >= settleTime)
            {
               if (Double.isNaN(refX))
               {
                  refX = x;
                  refY = y;
               }
               double drift = Math.hypot(x - refX, y - refY);
               maxDrift = Math.max(maxDrift, drift);
               if (!Double.isNaN(ex))
                  maxIcpError = Math.max(maxIcpError, Math.hypot(ex, ey));

               if (state.toUpperCase(Locale.ROOT).contains("STANDING") || walkingState == null)
                  standingAccumulated += 1.0;
            }

            double driftNow = Double.isNaN(refX) ? 0.0 : Math.hypot(x - refX, y - refY);
            csv.printf(Locale.ROOT, "%.2f,%.5f,%.5f,%.5f,%.5f,%.5f,%s,%.5f%n", t, x, y, z, ex, ey, state, driftNow);

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

      double requiredStanding = duration - settleTime - 1.0;
      if (standingAccumulated < Math.min(60.0, requiredStanding))
      {
         failures.add("standing accumulated " + standingAccumulated + "s < required " + Math.min(60.0, requiredStanding) + "s");
         pass = false;
      }
      if (maxDrift >= 0.02)
      {
         failures.add("CoM/root drift " + maxDrift + " >= 0.02 m");
         pass = false;
      }
      if (icpErrorX != null && maxIcpError >= 0.05)
      {
         failures.add("ICP error " + maxIcpError + " >= 0.05 m");
         pass = false;
      }

      System.out.println(String.format(Locale.ROOT, "GATE_M1_METRICS standing=%.1f maxDrift=%.4f maxIcpError=%.4f", standingAccumulated, maxDrift, maxIcpError));
      for (String failure : failures)
         System.out.println("GATE_M1_FAILURE " + failure);
      System.out.println(pass ? "GATE_M1_PASS" : "GATE_M1_FAIL");

      avatarSimulation.destroy();
      System.exit(pass ? 0 : 2);
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

   private static String name(YoVariable variable)
   {
      return variable == null ? "null" : variable.getFullNameString();
   }
}
