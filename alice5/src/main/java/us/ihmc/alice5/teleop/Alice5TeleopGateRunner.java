package us.ihmc.alice5.teleop;

import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * SIM-EXT V1 gate tool: drives the ALICE5 teleop sim (Alice5WalkingDemo with
 * -Dalice5.csg=joystick -Dalice5.realtime=true -Dcreate.yovariable.server=true) with a scripted
 * command sequence through the YoVariableServer change channel, without a physical gamepad.
 *
 * Sequence (each phase ramped at +/-0.15 m/s^2):
 *   step-in-place (6 s) -> forward 0.2 m/s (8 s) -> lateral 0.1 m/s (8 s)
 *   -> turn 0.3 rad/s (8 s) -> stop (6 s)  -- 36 s of continuous commanding (>= 30 s required).
 *
 * All scheduling and measurement use SIMULATION time (the YoVariableServer stream timestamp):
 * on Jetson 201 the headless sim runs at roughly 0.35x wall clock even with
 * setRealTimeRateSimulation(true) (which only caps at 1.0x), so wall-clock scheduling would
 * distort ramps, windows and measured velocities.
 *
 * Actual velocity is measured client-side from the streamed estimator root-joint state
 * (estimatedRootJointPositionX/Y + estimatedRootJointYaw): windowed finite difference over one
 * nominal step cycle (1.2 s), rotated into the body frame by the mid-window yaw.
 *
 * Verdict per phase (steady state = last 3 s): |mean - cmd| <= max(30% * |cmd|, floor)
 * with floors 0.03 m/s (linear) / 0.08 rad/s (angular). Plus: no fall (z >= 0.6 * z0), stream
 * alive, and explicit change-channel verification (walk=true survives on the server while the
 * client stops asserting it for 0.5 s).
 *
 * Output: CSV (cmd vs actual) + GATE_V1_PASS / GATE_V1_FAIL on stdout. Exit 0 pass, 2 fail, 3 setup.
 */
public class Alice5TeleopGateRunner
{
   private static final double CONTROL_DT = 0.02; // 50 Hz wall-clock write rate
   private static final double RAMP_LIMIT = 0.15; // m/s^2 and rad/s^2, in sim time
   private static final double VELOCITY_WINDOW = 1.2; // sim s, one nominal step cycle (swing 0.8 + transfer 0.4)
   private static final double STEADY_STATE_WINDOW = 3.0; // sim s, evaluated at the end of each phase
   private static final double LINEAR_FLOOR = 0.03; // m/s
   private static final double ANGULAR_FLOOR = 0.08; // rad/s
   private static final double RELATIVE_TOLERANCE = 0.30;

   record Phase(String name, double duration, double vx, double vy, double wz, boolean scored, char scoredAxis)
   {
   }

   record Sample(double t, double x, double y, double z, double yawUnwrapped)
   {
   }

   public static void main(String[] args) throws Exception
   {
      String host = "localhost";
      int port = TeleopYoVariableConnection.DEFAULT_PORT;
      String csvPath = "simext_v1_tracking.csv";
      double settleTime = 5.0;
      for (int i = 0; i < args.length - 1; i++)
      {
         if (args[i].equals("--server"))
            host = args[i + 1];
         else if (args[i].equals("--port"))
            port = Integer.parseInt(args[i + 1]);
         else if (args[i].equals("--csv"))
            csvPath = args[i + 1];
         else if (args[i].equals("--settle"))
            settleTime = Double.parseDouble(args[i + 1]);
      }

      List<Phase> phases = List.of(new Phase("stepInPlace", 6.0, 0.0, 0.0, 0.0, false, '-'),
                                   new Phase("forward", 8.0, 0.2, 0.0, 0.0, true, 'x'),
                                   new Phase("lateral", 8.0, 0.0, 0.1, 0.0, true, 'y'),
                                   new Phase("turn", 8.0, 0.0, 0.0, 0.3, true, 'w'),
                                   new Phase("stop", 6.0, 0.0, 0.0, 0.0, false, '-'));
      double totalDuration = phases.stream().mapToDouble(Phase::duration).sum();

      TeleopYoVariableConnection connection = new TeleopYoVariableConnection();
      try
      {
         connection.connect(host, port, 30.0);
      }
      catch (Exception e)
      {
         System.out.println("GATE_V1_FAILURE connect failed: " + e);
         System.out.println("GATE_V1_FAIL");
         System.exit(3);
      }

      if (!connection.hasCommandVariables() || !connection.hasStateVariables())
      {
         System.out.println("GATE_V1_FAILURE required yoVariables missing (see [teleop] resolution log above)");
         System.out.println("GATE_V1_FAIL");
         connection.disconnect();
         System.exit(3);
      }

      // wait for streaming data
      long waitStart = System.nanoTime();
      while (connection.getPacketCount() < 10)
      {
         Thread.sleep(100);
         if ((System.nanoTime() - waitStart) / 1e9 > 20.0)
         {
            System.out.println("GATE_V1_FAILURE no streamed data within 20 s");
            System.out.println("GATE_V1_FAIL");
            connection.disconnect();
            System.exit(3);
         }
      }
      System.out.println("[gate] stream alive (" + connection.getPacketCount() + " packets), settling " + settleTime + " s (wall)");
      Thread.sleep((long) (settleTime * 1000.0));

      double z0 = connection.getZ();
      System.out.println(String.format(Locale.ROOT, "[gate] initial state x=%.3f y=%.3f z=%.3f yaw=%.3f walkingState=%s",
                                       connection.getX(), connection.getY(), z0, connection.getYaw(), connection.getWalkingState()));
      System.out.println(String.format(Locale.ROOT, "[gate] CSG max velocities: vx=%.3f m/s vy=%.3f m/s wz=%.3f rad/s (achievable avg turn = wzMax/2 when turnMaxAngleInward=0)",
                                       connection.getMaxVelocityX(), connection.getMaxVelocityY(), connection.getMaxVelocityTurn()));

      List<String> failures = new ArrayList<>();
      boolean changeChannelVerified = false;
      double firstStepTime = Double.NaN;

      VelocityRamp rampVx = new VelocityRamp(RAMP_LIMIT);
      VelocityRamp rampVy = new VelocityRamp(RAMP_LIMIT);
      VelocityRamp rampWz = new VelocityRamp(RAMP_LIMIT);

      List<Sample> samples = new ArrayList<>(20000);
      double yawUnwrapped = connection.getYaw();
      double previousRawYaw = yawUnwrapped;

      // per-phase steady-state accumulators
      double[] steadySum = new double[phases.size()];
      int[] steadyCount = new int[phases.size()];

      long timestamp0 = connection.getLatestTimestampNanos();
      long wallStart = System.nanoTime();

      try (PrintWriter csv = new PrintWriter(csvPath))
      {
         csv.println("t,phase,cmdVx,cmdVy,cmdWz,x,y,z,yaw,bodyVx,bodyVy,yawRate,csgVx,csgVy,csgWz,walkReadback,walkingState");

         long tick = 0;
         boolean fall = false;
         double prevSimT = 0.0;

         while (true)
         {
            double simT = (connection.getLatestTimestampNanos() - timestamp0) / 1e9;
            if (simT > totalDuration)
               break;
            double dtSim = Math.max(0.0, simT - prevSimT);
            prevSimT = simT;

            // resolve phase by sim time
            int phaseIndex = 0;
            double phaseStart = 0.0;
            double acc = 0.0;
            for (int i = 0; i < phases.size(); i++)
            {
               if (simT < acc + phases.get(i).duration() || i == phases.size() - 1)
               {
                  phaseIndex = i;
                  phaseStart = acc;
                  break;
               }
               acc += phases.get(i).duration();
            }
            Phase phase = phases.get(phaseIndex);

            double cmdVx = rampVx.update(phase.vx(), dtSim);
            double cmdVy = rampVy.update(phase.vy(), dtSim);
            double cmdWz = rampWz.update(phase.wz(), dtSim);

            // Change-channel verification probe: in two 0.5 sim-s windows the client does NOT
            // assert walk; the streamed mirror then reflects the pure server-side value. If it
            // reads true, the earlier change request demonstrably landed on the server.
            boolean probeWindow = (simT >= 1.0 && simT < 1.5) || (simT >= 3.0 && simT < 3.5);
            if (probeWindow)
            {
               // assert velocities only; leave walk to the stream
               connection.sendCommandPhysical(connection.getWalkReadback(), cmdVx, cmdVy, cmdWz);
            }
            else
            {
               connection.sendCommandPhysical(true, cmdVx, cmdVy, cmdWz);
            }
            boolean probeCheckTick = (simT >= 1.45 && simT < 1.5) || (simT >= 3.45 && simT < 3.5);
            if (!changeChannelVerified && probeCheckTick && connection.getWalkReadback())
            {
               changeChannelVerified = true;
               System.out.println(String.format(Locale.ROOT,
                     "[gate] CHANGE_CHANNEL_VERIFIED walk=true persisted server-side during 0.5 s client write pause (simT=%.2f)", simT));
            }

            // sample state
            double x = connection.getX();
            double y = connection.getY();
            double z = connection.getZ();
            double rawYaw = connection.getYaw();
            double dyaw = rawYaw - previousRawYaw;
            if (dyaw > Math.PI)
               dyaw -= 2.0 * Math.PI;
            else if (dyaw < -Math.PI)
               dyaw += 2.0 * Math.PI;
            yawUnwrapped += dyaw;
            previousRawYaw = rawYaw;
            samples.add(new Sample(simT, x, y, z, yawUnwrapped));

            // windowed body-frame velocity over VELOCITY_WINDOW sim seconds
            double bodyVx = Double.NaN, bodyVy = Double.NaN, yawRate = Double.NaN;
            Sample now = samples.get(samples.size() - 1);
            Sample old = null;
            for (int i = samples.size() - 1; i >= 0; i--)
            {
               if (now.t() - samples.get(i).t() >= VELOCITY_WINDOW)
               {
                  old = samples.get(i);
                  break;
               }
            }
            if (old != null && now.t() - old.t() > 0.5 * VELOCITY_WINDOW)
            {
               double dt = now.t() - old.t();
               double vxWorld = (now.x() - old.x()) / dt;
               double vyWorld = (now.y() - old.y()) / dt;
               double yawMid = 0.5 * (now.yawUnwrapped() + old.yawUnwrapped());
               bodyVx = Math.cos(yawMid) * vxWorld + Math.sin(yawMid) * vyWorld;
               bodyVy = -Math.sin(yawMid) * vxWorld + Math.cos(yawMid) * vyWorld;
               yawRate = (now.yawUnwrapped() - old.yawUnwrapped()) / dt;
            }

            // steady-state accumulation (last STEADY_STATE_WINDOW sim s of scored phases)
            double phaseElapsed = simT - phaseStart;
            if (phase.scored() && phaseElapsed >= phase.duration() - STEADY_STATE_WINDOW && !Double.isNaN(bodyVx) && dtSim > 0.0)
            {
               double value = switch (phase.scoredAxis())
               {
                  case 'x' -> bodyVx;
                  case 'y' -> bodyVy;
                  case 'w' -> yawRate;
                  default -> Double.NaN;
               };
               steadySum[phaseIndex] += value;
               steadyCount[phaseIndex]++;
            }

            // first-step evidence
            String walkingState = connection.getWalkingState();
            if (Double.isNaN(firstStepTime) && (walkingState.equals("WALKING_LEFT_SUPPORT") || walkingState.equals("WALKING_RIGHT_SUPPORT")))
            {
               firstStepTime = simT;
               System.out.println(String.format(Locale.ROOT, "[gate] first step observed at simT=%.2f s (state=%s)", simT, walkingState));
            }

            csv.printf(Locale.ROOT, "%.3f,%s,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%b,%s%n",
                       simT, phase.name(), cmdVx, cmdVy, cmdWz, x, y, z, rawYaw, bodyVx, bodyVy, yawRate,
                       connection.getCsgEffectiveVelocityX(), connection.getCsgEffectiveVelocityY(), connection.getCsgEffectiveTurningVelocity(),
                       connection.getWalkReadback(), walkingState);

            if (tick % 100 == 0)
            {
               double wallT = (System.nanoTime() - wallStart) / 1e9;
               System.out.println(String.format(Locale.ROOT,
                                                "[gate] simT=%5.1f wall=%5.1f (%.2fx) phase=%-12s cmd=(%.2f,%.2f,%.2f) act=(%.2f,%.2f,%.2f) z=%.3f state=%s",
                                                simT, wallT, wallT > 0 ? simT / wallT : 0.0, phase.name(), cmdVx, cmdVy, cmdWz,
                                                bodyVx, bodyVy, yawRate, z, walkingState));
            }

            // safety checks
            if (!Double.isNaN(z) && !Double.isNaN(z0) && z < 0.6 * z0)
            {
               failures.add(String.format(Locale.ROOT, "FALL at simT=%.2f (z=%.3f, z0=%.3f)", simT, z, z0));
               fall = true;
               break;
            }
            if (connection.getDataAgeSeconds() > 5.0)
            {
               failures.add(String.format(Locale.ROOT, "stream stalled at simT=%.2f (age=%.1f s wall)", simT, connection.getDataAgeSeconds()));
               break;
            }

            tick++;
            long targetNanos = wallStart + (long) (tick * CONTROL_DT * 1e9);
            long sleepNanos = targetNanos - System.nanoTime();
            if (sleepNanos > 0)
               Thread.sleep(sleepNanos / 1_000_000, (int) (sleepNanos % 1_000_000));
         }

         // final stop
         connection.sendEStop();
         Thread.sleep(500);
         connection.sendEStop();

         if (fall)
            System.out.println("[gate] aborted by fall");
      }

      // verdict
      if (!changeChannelVerified)
         failures.add("change channel not verified (walk=true did not persist server-side during client write pause)");
      if (Double.isNaN(firstStepTime))
         failures.add("robot never took a step (walkingCurrentState never entered single support)");

      System.out.println("[gate] steady-state tracking (last " + STEADY_STATE_WINDOW + " sim-s of each scored phase):");
      for (int i = 0; i < phases.size(); i++)
      {
         Phase phase = phases.get(i);
         if (!phase.scored())
            continue;
         double cmd = switch (phase.scoredAxis())
         {
            case 'x' -> phase.vx();
            case 'y' -> phase.vy();
            case 'w' -> phase.wz();
            default -> Double.NaN;
         };
         double floor = phase.scoredAxis() == 'w' ? ANGULAR_FLOOR : LINEAR_FLOOR;
         if (steadyCount[i] == 0)
         {
            failures.add(phase.name() + ": no steady-state samples (run aborted early?)");
            continue;
         }
         double mean = steadySum[i] / steadyCount[i];
         double error = Math.abs(mean - cmd);
         double tolerance = Math.max(RELATIVE_TOLERANCE * Math.abs(cmd), floor);
         boolean ok = error <= tolerance;
         System.out.println(String.format(Locale.ROOT, "[gate]   %-8s cmd=%.3f actual=%.4f err=%.4f tol=%.4f (n=%d) -> %s",
                                          phase.name(), cmd, mean, error, tolerance, steadyCount[i], ok ? "OK" : "FAIL"));
         if (!ok)
            failures.add(String.format(Locale.ROOT, "%s tracking: actual %.4f vs cmd %.3f (err %.4f > tol %.4f)",
                                       phase.name(), mean, cmd, error, tolerance));
      }

      System.out.println(String.format(Locale.ROOT,
                                       "GATE_V1_METRICS durationCommandedSimSeconds=%.1f firstStep=%.2f changeChannelVerified=%b packets=%d",
                                       totalDuration, firstStepTime, changeChannelVerified, connection.getPacketCount()));
      for (String failure : failures)
         System.out.println("GATE_V1_FAILURE " + failure);
      boolean pass = failures.isEmpty();
      System.out.println(pass ? "GATE_V1_PASS" : "GATE_V1_FAIL");

      connection.disconnect();
      System.exit(pass ? 0 : 2);
   }
}
