package us.ihmc.alice5.teleop;

/**
 * SIM-EXT V1: simple slew-rate limiter for velocity commands (default +/-0.15 m/s^2 per spec).
 */
public class VelocityRamp
{
   private final double maxRatePerSecond;
   private double current;

   public VelocityRamp(double maxRatePerSecond)
   {
      this.maxRatePerSecond = maxRatePerSecond;
   }

   public double update(double target, double dt)
   {
      double maxDelta = maxRatePerSecond * dt;
      double delta = target - current;
      if (delta > maxDelta)
         delta = maxDelta;
      else if (delta < -maxDelta)
         delta = -maxDelta;
      current += delta;
      return current;
   }

   /** Immediate override (E-stop path bypasses ramping). */
   public void reset(double value)
   {
      current = value;
   }

   public double getCurrent()
   {
      return current;
   }
}
