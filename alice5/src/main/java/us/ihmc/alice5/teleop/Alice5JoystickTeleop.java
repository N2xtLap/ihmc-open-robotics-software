package us.ihmc.alice5.teleop;

import net.java.games.input.Component;
import net.java.games.input.Event;
import us.ihmc.tools.inputDevices.joystick.Joystick;

import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * SIM-EXT V1 teleop client (run on the dev PC): connects to the ALICE5 teleop sim
 * (Alice5WalkingDemo with -Dalice5.csg=joystick -Dalice5.realtime=true
 * -Dcreate.yovariable.server=true) and pushes ContinuousStepGenerator velocity commands through
 * the YoVariableServer change channel.
 *
 * Input: IHMC Joystick library (net.java.games.input) if a gamepad is present, otherwise keyboard
 * fallback (raw console).
 *
 * Gamepad mapping: left stick Y = forward, left stick X = lateral, right stick X = turn,
 * button 0/A = walk on, button 1/B = E-stop (walk off + zero immediately).
 * Keyboard mapping: w/s = forward +/-, a/d = lateral left/right, q/e = turn left/right,
 * g = walk on, space = E-stop, z = zero velocities, x = quit.
 *
 * Safety: deadzone 0.1 on joystick axes, command ramp limited to +/-0.15 m/s^2 (rad/s^2),
 * E-stop bypasses the ramp. Status printed at 1 Hz.
 *
 * Usage: Alice5JoystickTeleop --server <host> [--port 8008] [--vxmax 0.25] [--vymax 0.10] [--wzmax 0.30]
 */
public class Alice5JoystickTeleop
{
   private static final double CONTROL_DT = 0.02; // 50 Hz
   private static final double RAMP_LIMIT = 0.15;
   private static final double DEADZONE = 0.1;
   private static final double KEYBOARD_LINEAR_STEP = 0.05;
   private static final double KEYBOARD_ANGULAR_STEP = 0.05;

   // targets written by the input thread, consumed by the control loop
   private volatile double targetVx = 0.0;
   private volatile double targetVy = 0.0;
   private volatile double targetWz = 0.0;
   private volatile boolean walkEnabled = false;
   private final AtomicBoolean eStop = new AtomicBoolean(false);
   private final AtomicBoolean quit = new AtomicBoolean(false);

   private final double vxMax;
   private final double vyMax;
   private final double wzMax;

   public Alice5JoystickTeleop(double vxMax, double vyMax, double wzMax)
   {
      this.vxMax = vxMax;
      this.vyMax = vyMax;
      this.wzMax = wzMax;
   }

   public static void main(String[] args) throws Exception
   {
      String host = "localhost";
      int port = TeleopYoVariableConnection.DEFAULT_PORT;
      double vxMax = 0.25, vyMax = 0.10, wzMax = 0.30;
      for (int i = 0; i < args.length - 1; i++)
      {
         if (args[i].equals("--server"))
            host = args[i + 1];
         else if (args[i].equals("--port"))
            port = Integer.parseInt(args[i + 1]);
         else if (args[i].equals("--vxmax"))
            vxMax = Double.parseDouble(args[i + 1]);
         else if (args[i].equals("--vymax"))
            vyMax = Double.parseDouble(args[i + 1]);
         else if (args[i].equals("--wzmax"))
            wzMax = Double.parseDouble(args[i + 1]);
      }

      Alice5JoystickTeleop teleop = new Alice5JoystickTeleop(vxMax, vyMax, wzMax);

      TeleopYoVariableConnection connection = new TeleopYoVariableConnection();
      connection.connect(host, port, 30.0);
      if (!connection.hasCommandVariables())
      {
         System.out.println("[teleop] CSG command variables missing -- is the demo running with -Dalice5.csg=joystick?");
         connection.disconnect();
         System.exit(3);
      }

      boolean joystickMode = teleop.tryStartJoystick();
      if (!joystickMode)
         teleop.startKeyboard();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         try
         {
            connection.sendEStop();
         }
         catch (Exception ignored)
         {
         }
         restoreTerminal();
      }));

      teleop.controlLoop(connection);

      connection.sendEStop();
      Thread.sleep(300);
      connection.disconnect();
      restoreTerminal();
      System.exit(0);
   }

   private void controlLoop(TeleopYoVariableConnection connection) throws InterruptedException
   {
      VelocityRamp rampVx = new VelocityRamp(RAMP_LIMIT);
      VelocityRamp rampVy = new VelocityRamp(RAMP_LIMIT);
      VelocityRamp rampWz = new VelocityRamp(RAMP_LIMIT);

      long t0 = System.nanoTime();
      long tick = 0;
      while (!quit.get())
      {
         if (eStop.get())
         {
            walkEnabled = false;
            targetVx = 0.0;
            targetVy = 0.0;
            targetWz = 0.0;
            rampVx.reset(0.0);
            rampVy.reset(0.0);
            rampWz.reset(0.0);
            connection.sendEStop();
            eStop.set(false);
            System.out.println("[teleop] E-STOP: walk=false, commands zeroed");
         }
         else
         {
            double cmdVx = rampVx.update(clamp(targetVx, vxMax), CONTROL_DT);
            double cmdVy = rampVy.update(clamp(targetVy, vyMax), CONTROL_DT);
            double cmdWz = rampWz.update(clamp(targetWz, wzMax), CONTROL_DT);
            connection.sendCommandPhysical(walkEnabled, cmdVx, cmdVy, cmdWz);
         }

         if (tick % 50 == 0)
         {
            System.out.println(String.format(Locale.ROOT,
                  "[teleop] walk=%b cmd=(%.2f, %.2f, %.2f) target=(%.2f, %.2f, %.2f) pos=(%.2f, %.2f, %.2f) yaw=%.2f state=%s age=%.2fs",
                  walkEnabled, rampVx.getCurrent(), rampVy.getCurrent(), rampWz.getCurrent(),
                  targetVx, targetVy, targetWz,
                  connection.getX(), connection.getY(), connection.getZ(), connection.getYaw(),
                  connection.getWalkingState(), connection.getDataAgeSeconds()));
            if (connection.getDataAgeSeconds() > 3.0)
               System.out.println("[teleop] WARNING: stream stalled");
         }

         tick++;
         long targetNanos = t0 + (long) (tick * CONTROL_DT * 1e9);
         long sleepNanos = targetNanos - System.nanoTime();
         if (sleepNanos > 0)
            Thread.sleep(sleepNanos / 1_000_000, (int) (sleepNanos % 1_000_000));
      }
      System.out.println("[teleop] quitting (walk=false sent)");
   }

   private static double clamp(double value, double max)
   {
      return Math.max(-max, Math.min(max, value));
   }

   private static double applyDeadzone(double value)
   {
      if (Math.abs(value) < DEADZONE)
         return 0.0;
      // rescale so output is continuous at the deadzone edge
      return Math.signum(value) * (Math.abs(value) - DEADZONE) / (1.0 - DEADZONE);
   }

   private boolean tryStartJoystick()
   {
      try
      {
         Joystick joystick = new Joystick();
         joystick.addJoystickEventListener(event -> processJoystickEvent(event));
         System.out.println("[teleop] gamepad found: left stick = fwd/lat, right stick X = turn, btn0/A = walk, btn1/B = E-stop");
         return true;
      }
      catch (Throwable e)
      {
         System.out.println("[teleop] no gamepad (" + e.getMessage() + ") -- keyboard fallback: w/s fwd, a/d lat, q/e turn, g walk, SPACE e-stop, z zero, x quit");
         return false;
      }
   }

   private void processJoystickEvent(Event event)
   {
      Component.Identifier identifier = event.getComponent().getIdentifier();
      float value = event.getValue();
      if (identifier == Component.Identifier.Axis.Y)
         targetVx = applyDeadzone(-value) * vxMax; // stick up = negative -> forward positive
      else if (identifier == Component.Identifier.Axis.X)
         targetVy = applyDeadzone(-value) * vyMax; // stick left = negative -> left (+y) positive
      else if (identifier == Component.Identifier.Axis.RX)
         targetWz = applyDeadzone(-value) * wzMax; // stick left = negative -> CCW (+) turn
      else if (identifier instanceof Component.Identifier.Button)
      {
         String name = identifier.getName();
         boolean pressed = value > 0.5f;
         if (pressed && (name.equals("0") || name.equalsIgnoreCase("A")))
         {
            walkEnabled = true;
            System.out.println("[teleop] walk ON");
         }
         else if (pressed && (name.equals("1") || name.equalsIgnoreCase("B")))
         {
            eStop.set(true);
         }
      }
   }

   private void startKeyboard()
   {
      setTerminalRaw();
      Thread keyboardThread = new Thread(() ->
      {
         try
         {
            while (!quit.get())
            {
               int c = System.in.read();
               if (c < 0)
                  break;
               switch (Character.toLowerCase((char) c))
               {
                  case 'w' -> targetVx = clamp(targetVx + KEYBOARD_LINEAR_STEP, vxMax);
                  case 's' -> targetVx = clamp(targetVx - KEYBOARD_LINEAR_STEP, vxMax);
                  case 'a' -> targetVy = clamp(targetVy + KEYBOARD_LINEAR_STEP, vyMax);
                  case 'd' -> targetVy = clamp(targetVy - KEYBOARD_LINEAR_STEP, vyMax);
                  case 'q' -> targetWz = clamp(targetWz + KEYBOARD_ANGULAR_STEP, wzMax);
                  case 'e' -> targetWz = clamp(targetWz - KEYBOARD_ANGULAR_STEP, wzMax);
                  case 'g' ->
                  {
                     walkEnabled = true;
                     System.out.println("[teleop] walk ON");
                  }
                  case ' ' -> eStop.set(true);
                  case 'z' ->
                  {
                     targetVx = 0.0;
                     targetVy = 0.0;
                     targetWz = 0.0;
                  }
                  case 'x' -> quit.set(true);
                  default ->
                  {
                  }
               }
            }
         }
         catch (Exception e)
         {
            System.out.println("[teleop] keyboard thread: " + e);
         }
      }, "teleop-keyboard");
      keyboardThread.setDaemon(true);
      keyboardThread.start();
   }

   private static void setTerminalRaw()
   {
      try
      {
         new ProcessBuilder("/bin/sh", "-c", "stty -icanon -echo min 1 < /dev/tty").inheritIO().start().waitFor();
      }
      catch (Exception e)
      {
         System.out.println("[teleop] raw terminal mode unavailable (" + e.getMessage() + ") -- press ENTER after each key");
      }
   }

   private static void restoreTerminal()
   {
      try
      {
         new ProcessBuilder("/bin/sh", "-c", "stty sane < /dev/tty").inheritIO().start().waitFor();
      }
      catch (Exception ignored)
      {
      }
   }
}
