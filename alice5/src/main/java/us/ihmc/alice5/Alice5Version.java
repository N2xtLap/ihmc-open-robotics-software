package us.ihmc.alice5;

import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.alice5.parameters.model.Alice5URDFParameters;
import us.ihmc.alice5.parameters.model.HumanoidURDFParameterInterface;

import java.util.Arrays;
import java.util.Collection;

public enum Alice5Version implements Alice5VersionInterface
{
   V1_FULL_ROBOT(Arrays.asList(Alice5URDFParameters.URDF_FULL_BODY));

   private final Collection<String> urdfModelPath;

   private Alice5JointMap jointMap;
   private Alice5PhysicalProperties physicalProperties;
   private Alice5SensorInformation sensorInformation;
   private Alice5URDFParameters urdfParameters;

   Alice5Version(Collection<String> urdfModelPath)
   {
      this.urdfModelPath = urdfModelPath;
   }

   @Override
   public Collection<String> getModelPath()
   {
      return urdfModelPath;
   }

   @Override
   public Alice5JointMap getJointMap()
   {
      if (jointMap == null)
         jointMap = new Alice5JointMap(getPhysicalProperties());
      return jointMap;
   }

   @Override
   public Alice5SensorInformation getSensorInformation()
   {
      if (sensorInformation == null)
         sensorInformation = new Alice5SensorInformation(this);
      return sensorInformation;
   }

   @Override
   public Alice5PhysicalProperties getPhysicalProperties()
   {
      if (physicalProperties == null)
         physicalProperties = new Alice5PhysicalProperties();
      return physicalProperties;
   }

   @Override
   public HumanoidURDFParameterInterface getURDFParameters()
   {
      if (urdfParameters == null)
         urdfParameters = new Alice5URDFParameters(this);
      return urdfParameters;
   }
}
