package us.ihmc.alice5.parameters.controller;

import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class Alice5ToeOffParameters extends ToeOffParameters
{
   private final Alice5PhysicalProperties alice5PhysicalProperties;

   public Alice5ToeOffParameters(Alice5PhysicalProperties alice5PhysicalProperties)
   {
      this.alice5PhysicalProperties = alice5PhysicalProperties;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return alice5PhysicalProperties.getFootLengthForControl();
   }

   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   @Override
   public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
   {
      return true;
   }


   // TODO we should investigate turning this on on hardware
   //   @Override
   //   public boolean doToeOffWhenHittingTrailingKneeLowerLimit()
   //   {
   //      return true;
   //   }

   @Override
   public double getKneeLowerLimitToTriggerToeOff()
   {
      return 0.4;
   }

}
