package us.ihmc.alice5.parameters.planning;

import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class Alice5FootstepPlannerParameters extends StoredPropertySet implements DefaultFootstepPlannerParametersBasics
{
   public Alice5FootstepPlannerParameters()
   {
      this("");
   }

   public Alice5FootstepPlannerParameters(String versionSuffix)
   {
      super(DefaultFootstepPlannerParameters.keys, Alice5FootstepPlannerParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(DefaultFootstepPlannerParameters.keys, Alice5FootstepPlannerParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
