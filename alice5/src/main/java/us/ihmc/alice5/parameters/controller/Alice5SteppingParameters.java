package us.ihmc.alice5.parameters.controller;

import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class Alice5SteppingParameters implements SteppingParameters
{
   protected final Alice5PhysicalProperties alice5PhysicalProperties;

   public Alice5SteppingParameters(Alice5PhysicalProperties alice5PhysicalProperties)
   {
      this.alice5PhysicalProperties = alice5PhysicalProperties;
   }

   @Override
   public double getFootForwardOffset()
   {
      return alice5PhysicalProperties.getFootForwardForControl();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return alice5PhysicalProperties.getFootBackForControl();
   }

   @Override
   public double getInPlaceWidth()
   {
      // TODO Needs tune up.
      return 0.22;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.7;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.12;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.8;
   }

   @Override
   public double getDefaultStepLength()
   {
      // TODO Needs tune up.
      return 0.4;
   }

   @Override
   public double getMaxStepUp()
   {
      // TODO Needs tune up.
      return 0.25;
   }

   @Override
   public double getMaxStepDown()
   {
      // TODO Needs tune up.
      return 0.2;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return 0.65;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      // TODO Needs tune up.
      return 0.0;
   }

   @Override
   public double getFootWidth()
   {
      return alice5PhysicalProperties.getFootWidthForControl();
   }

   @Override
   public double getToeWidth()
   {
      return alice5PhysicalProperties.getToeWidthForControl();
   }

   @Override
   public double getFootLength()
   {
      return alice5PhysicalProperties.getFootLengthForControl();
   }

   @Override
   public double getActualFootWidth()
   {
      return alice5PhysicalProperties.getActualFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return alice5PhysicalProperties.getActualFootLength();
   }
}
