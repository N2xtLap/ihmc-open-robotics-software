package us.ihmc.alice5.parameters.controller;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;

public class Alice5ICPControllerParameters extends ICPControllerParameters
{
   @Override
   public double getSafeCoPDistanceToEdge()
   {
      // T1 terrain runs (-Dalice5.safeCopEdge): keep the CoP further inside the support polygon so
      // cell-edge partial contacts are not loaded to the rim. Upstream default 0.002.
      return Double.parseDouble(System.getProperty("alice5.safeCopEdge", "0.002"));
   }

   private FeedbackAlphaCalculator feedbackAlphaCalculator = null;
   private FeedForwardAlphaCalculator feedForwardAlphaCalculator = null;

   @Override
   public double getFeedbackForwardWeight()
   {
      // TODO Needs tune up.
      return 0.5;
   }

   @Override
   public double getFeedbackLateralWeight()
   {
      // TODO Needs tune up.
      return 0.5;
   }

   @Override
   public double getFeedbackRateWeight()
   {
      // TODO Needs tune up.
      return 1e-8;
   }

   @Override
   public ICPControlGainsReadOnly getICPFeedbackGains()
   {
      ICPControlGains gains = new ICPControlGains();
      // SIM-EXT ARM-1: kp 2.0/2.5 -> 2.5/3.0. With the arms position-held (Alice5JointMap.getHandName
      // fix) the implicit reaction-mass slack the QP used to get from the 6 uncontrolled arm joints is
      // gone and the M2 script fell at the side-step -> stop transfer (icpErrY 0.035 -> 0.099 at t=44).
      // Slightly stronger ICP feedback restores the lateral recovery margin; QP-side arm weight sweeps
      // (1.0/0.5/0.1) and angular-z momentum weight (0.1 -> 0.05) were inert (identical trajectories).
      gains.setKpOrthogonalToMotion(2.5);
      gains.setKpParallelToMotion(3.0);
      gains.setKi(2.0);
      return gains;
   }

   @Override
   public double getDynamicsObjectiveWeight()
   {
      return 10000.0;
   }

   @Override
   public double getAngularMomentumMinimizationWeight()
   {
      // TODO Needs tune up.
      return 10.0;
   }

   @Override
   public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   @Override
   public boolean getUseICPControlPolygons()
   {
      return false;
   }

   @Override
   public boolean useAngularMomentum()
   {
      return true;
   }

   @Override
   public boolean getUseHeuristicICPController()
   {
      return false;
   }

   @Override
   public double getPureFeedbackErrorThreshold()
   {
      return 0.06;
   }

   @Override
   public boolean useSmartICPIntegrator()
   {
      return true;
   }

   @Override
   public double getICPVelocityThresholdForStuck()
   {
      return 0.06;
   }

   @Override
   public double getFeedbackDirectionWeight()
   {
      return 1e6;
   }

   @Override
   public FeedForwardAlphaCalculator getFeedForwardAlphaCalculator()
   {
      return feedForwardAlphaCalculator;
   }

   @Override
   public FeedbackAlphaCalculator getFeedbackAlphaCalculator()
   {
      return feedbackAlphaCalculator;
   }
}
