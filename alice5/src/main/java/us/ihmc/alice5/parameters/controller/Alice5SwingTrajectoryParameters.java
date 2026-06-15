package us.ihmc.alice5.parameters.controller;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class Alice5SwingTrajectoryParameters extends SwingTrajectoryParameters
{
   @Override
   public double getFinalCoMVelocityInjectionRatio()
   {
      return 0.5;
   }

   @Override
   public double getFinalCoMAccelerationInjectionRatio()
   {
      return 0.0;
   }

   @Override
   public boolean addOrientationMidpointForObstacleClearance()
   {
      return true;
   }

   @Override
   public Tuple3DReadOnly getTouchdownVelocityWeight()
   {
      // SIM-EXT T2 sweep hook: z weight is INF by default (controller rigidly tracks the touchdown
      // velocity). -Dalice5.touchdownVelWeightZ lets a run trade that for a finite weight.
      String wz = System.getProperty("alice5.touchdownVelWeightZ", "");
      double weightZ = wz.isEmpty() ? Double.POSITIVE_INFINITY : Double.parseDouble(wz);
      return new Vector3D(30.0, 30.0, weightZ);
   }

   @Override
   public double getDefaultSwingHeight()
   {
      // SIM-EXT T2: lowered 0.09 -> 0.04 to soften flat-ground touchdown. The sole-frame contact
      // |vz| is set by the swing arc geometry (height/time), NOT the touchdown velocity/accel
      // setpoints (contact happens mid-descent, before the trajectory's terminal phase, so those
      // setpoints never reach the contact point -- see docs/gates/SIMEXT_T2.md). A lower arc means
      // a gentler descent: contact |vz| 0.54 -> 0.31 m/s (-43%), M2 preserved. Floor is
      // getMinSwingHeight()=0.025. Terrain/T1 is unaffected: it sets swingHeightCSG=0.10 explicitly
      // (Alice5TerrainWalkingDemo), overriding this default. -Dalice5.swingHeight overrides for sweeps.
      return Double.parseDouble(System.getProperty("alice5.swingHeight", "0.04"));
   }

   @Override
   public double getMinSwingHeight()
   {
      return 0.025;
   }

   @Override
   public double getMaxSwingHeight()
   {
      // TODO Needs tune up.
      return 0.35;
   }

   @Override
   public boolean useInitialToeHeight()
   {
      return true;
   }

   @Override
   public boolean useFinalHeelHeight()
   {
      return true;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0.0; // TODO Tune me up!
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      // T1 terrain runs (-Dalice5.touchdownVel): faster downward probe after the planned touchdown
      // z, so a blind step-down (-3 cm cell) finds ground sooner. Flat-ground default -0.1.
      return Double.parseDouble(System.getProperty("alice5.touchdownVel", "-0.1"));
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      // SIM-EXT T2: downward accel commanded at touchdown. The original -2.0 ("Needs tune up")
      // drives the foot to accelerate INTO the ground (measured sole contact |vz| ~0.54 m/s, ~5x
      // the -0.1 touchdown velocity). -Dalice5.touchdownAccel sweeps it; 0.0 removes the downward
      // push so the swing trajectory's terminal phase can bleed off velocity before contact.
      return Double.parseDouble(System.getProperty("alice5.touchdownAccel", "-2.0"));
   }

   @Override
   public double getBlindFootstepsHeightOffset()
   {
      return 0.0;
   }

   @Override
   public double getDefaultSwingStepUpHeight()
   {
      return 0.13;
   }

   @Override
   public double getDefaultSwingStepDownHeight()
   {
      return 0.10;
   }

   @Override
   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return 0.1;
   }

   @Override
   public double[] getSwingStepUpWaypointProportions()
   {
      return new double[] {0.05, 0.80};
   }

   @Override
   public double[] getSwingStepDownWaypointProportions()
   {
      return new double[] {0.20, 0.95};
   }

   @Override
   public double getFirstWaypointHeightFactorForSteppingUp()
   {
      return 1.5 / 3.0;
   }

   @Override
   public double getSecondWaypointHeightFactorForSteppingDown()
   {
      return 0.5;
   }

   /** {@inheritDoc} **/
   @Override
   public boolean addFootPitchToAvoidHeelStrikeWhenSteppingForwardAndDown()
   {
      return true;
   }
}