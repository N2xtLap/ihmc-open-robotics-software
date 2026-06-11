package us.ihmc.alice5.parameters.planning;

import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.tools.property.StoredPropertySet;

public class Alice5VisibilityGraphParameters extends StoredPropertySet implements VisibilityGraphsParametersBasics
{
   public Alice5VisibilityGraphParameters()
   {
      this("");
   }

   public Alice5VisibilityGraphParameters(String versionSuffix)
   {
      super(VisibilityGraphParametersKeys.keys, Alice5VisibilityGraphParameters.class, versionSuffix);
      loadUnsafe();
   }

   /** Use this to update and fix the INI file */
   public static void main(String[] args)
   {
      StoredPropertySet storedPropertySet = new StoredPropertySet(VisibilityGraphParametersKeys.keys, Alice5VisibilityGraphParameters.class);
      storedPropertySet.loadUnsafe();
      storedPropertySet.save();
   }
}
