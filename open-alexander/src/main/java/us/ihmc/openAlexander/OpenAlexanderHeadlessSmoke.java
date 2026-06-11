package us.ihmc.openAlexander;

/**
 * Headless smoke test for M0: instantiates the full open-alexander controller stack
 * (robot model, WBC, walking controller state) and runs the warmup ticks without GUI.
 */
public class OpenAlexanderHeadlessSmoke extends AlexanderControllerWarmup
{
   private void runHeadless()
   {
      runWarmup();
   }

   public static void main(String[] args)
   {
      long start = System.nanoTime();
      OpenAlexanderHeadlessSmoke smoke = new OpenAlexanderHeadlessSmoke();
      System.out.println("SMOKE_CONTROLLER_SETUP_OK elapsed_ms=" + (System.nanoTime() - start) / 1000000);
      smoke.runHeadless();
      System.out.println("SMOKE_OK elapsed_ms=" + (System.nanoTime() - start) / 1000000);
      System.exit(0);
   }
}
