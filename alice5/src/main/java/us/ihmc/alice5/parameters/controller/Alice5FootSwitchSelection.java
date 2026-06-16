package us.ihmc.alice5.parameters.controller;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.KinematicsBasedFootSwitchFactory;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.sensors.FootSwitchFactory;

/**
 * Foot-switch factory selection for ALICE5 (SIM-EXT FT).
 * <p>
 * ALICE5's feet carry four corner loadcells used as a synthesized F/T sensor: the robot side sums
 * the vertical cell forces and the center of pressure into a sole-frame wrench (Fz, Mx, My; the
 * loadcells cannot observe Fx/Fy/Mz, so those stay zero) that the shared-memory bridge delivers to
 * the per-foot ForceSensorData. The default foot switch is therefore wrench based, which lets the
 * estimator and the walking state machine use the measured ground reaction for contact and CoP.
 * <p>
 * A kinematics-based fall-back is kept for when the loadcell wrench is unavailable or suspect. The
 * selection is per process through the system property {@code -Dalice5.footswitch=wrench|kinematic}
 * (default {@code wrench}). Both the walking controller and the state estimator read the same
 * property so they always agree on which switch is in use.
 */
public final class Alice5FootSwitchSelection
{
   /** {@code -Dalice5.footswitch=wrench|kinematic}; default wrench (loadcell ground reaction). */
   public static final String PROPERTY = "alice5.footswitch";

   // Wrench thresholds: a fraction of body weight for first contact, a higher fraction that admits
   // contact even without a valid CoP. Held here so the controller and estimator factories match.
   private static final double WRENCH_CONTACT_THRESHOLD_FORCE = 50.0;
   private static final double WRENCH_COP_THRESHOLD_DISTANCE = 4.0e-3;
   private static final double WRENCH_SECOND_CONTACT_THRESHOLD_FORCE = 75.0;

   // Kinematic fall-back: sole-frame height below which the foot counts as on the ground. ALICE5 has
   // never run on the kinematic switch (wrench has been the default since M1), so there is no tuned
   // value to inherit; this is a conservative swing-clearance-scale guess. [HUMAN VERIFY]
   private static final double KINEMATIC_CONTACT_THRESHOLD_HEIGHT = 0.02;

   private Alice5FootSwitchSelection()
   {
   }

   public static boolean useWrench()
   {
      String mode = System.getProperty(PROPERTY, "wrench").trim().toLowerCase();
      if (mode.equals("kinematic"))
         return false;
      if (!mode.equals("wrench"))
         LogTools.warn("Unknown " + PROPERTY + "=" + mode + ", defaulting to wrench.");
      return true;
   }

   public static FootSwitchFactory createFootSwitchFactory()
   {
      if (useWrench())
      {
         WrenchBasedFootSwitchFactory factory = new WrenchBasedFootSwitchFactory();
         factory.setDefaultContactThresholdForce(WRENCH_CONTACT_THRESHOLD_FORCE);
         factory.setDefaultCoPThresholdDistance(WRENCH_COP_THRESHOLD_DISTANCE);
         factory.setDefaultSecondContactThresholdForceIgnoringCoP(WRENCH_SECOND_CONTACT_THRESHOLD_FORCE);
         return factory;
      }

      KinematicsBasedFootSwitchFactory factory = new KinematicsBasedFootSwitchFactory();
      factory.setDefaultContactThresholdHeight(KINEMATIC_CONTACT_THRESHOLD_HEIGHT);
      return factory;
   }
}
