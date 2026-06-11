package us.ihmc.alice5.parameters.planning;

import us.ihmc.footstepPlanning.LocomotionParameters;

public class Alice5LocomotionParameters extends LocomotionParameters
{
   public Alice5LocomotionParameters()
   {
      super(Alice5LocomotionParameters.class);
      loadUnsafe();
   }
}
