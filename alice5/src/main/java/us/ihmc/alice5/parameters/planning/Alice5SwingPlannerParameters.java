package us.ihmc.alice5.parameters.planning;

import us.ihmc.footstepPlanning.swing.SwingPlannerParameterKeys;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class Alice5SwingPlannerParameters extends StoredPropertySet implements SwingPlannerParametersBasics
{
   public Alice5SwingPlannerParameters()
   {
      this("");
   }

   public Alice5SwingPlannerParameters(String versionSuffix)
   {
      super(SwingPlannerParameterKeys.keys, Alice5SwingPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   public static void main(String[] args)
   {
      Alice5SwingPlannerParameters parameters = new Alice5SwingPlannerParameters();
      parameters.save();
   }
}
