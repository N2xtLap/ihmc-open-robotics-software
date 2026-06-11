package us.ihmc.alice5;

import org.apache.commons.lang3.SystemUtils;
import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.alice5.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.DefaultLogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class Alice5ModelFactory
{
   private final String[] resourceModelsToBeLogged;
   private final HumanoidURDFParameterInterface urdfParameters;
   private final RobotContactPointParameters<RobotSide> contactPointParameters;
   private RobotDefinition simulationRobotDefinition;
   private final Alice5VersionInterface version;
   private RobotDefinition controllerRobotDefinition;
   private final Alice5JointMap jointMap;
   private final Consumer<RobotDefinition> robotDefinitionMutator;

   public Alice5ModelFactory(Alice5VersionInterface version,
                             Alice5JointMap jointMap,
                             RobotContactPointParameters<RobotSide> contactPointParameters,
                             Consumer<RobotDefinition> robotDefinitionMutator)
   {
      this.version = version;
      this.jointMap = jointMap;
      this.contactPointParameters = contactPointParameters;
      this.robotDefinitionMutator = robotDefinitionMutator;

      urdfParameters = version.getURDFParameters();
      resourceModelsToBeLogged = urdfParameters.getLoggedResources();
   }

   public LogModelProvider createLogModelProvider()
   {
      if (SystemUtils.OS_NAME.contains("Windows"))
      {
         for (int i = 0; i < resourceModelsToBeLogged.length; i++)
         {
            resourceModelsToBeLogged[i] = resourceModelsToBeLogged[i].replace('/', '\\');
         }
      }

      Predicate<String> filter = resourcePath ->
      {
         for (String model : resourceModelsToBeLogged)
         {
            if (resourcePath.startsWith(model))
               return true;
         }
         return false;
      };

      Class<?> clazz = URDFModel.class;
      return new DefaultLogModelProvider<>(clazz, urdfParameters.getURDFModelName(), urdfParameters.getURDFAsInputStream(), filter,
                                           urdfParameters.getResourceDirectories());
   }

   public RobotDefinition getSCS1RobotDefinition()
   {
      if (simulationRobotDefinition == null)
      {
         URDFTools.URDFParserProperties parserProperties = new URDFTools.URDFParserProperties();
         parserProperties.setTransformToZUp(false);
         simulationRobotDefinition = RobotDefinitionLoader.loadURDFModel(urdfParameters.getURDFAsInputStream(),
                                                                         Arrays.asList(urdfParameters.getResourceDirectories()),
                                                                         getClass().getClassLoader(),
                                                                         urdfParameters.getURDFModelName(),
                                                                         contactPointParameters,
                                                                         jointMap,
                                                                         true,
                                                                         parserProperties);
         if (version.getSensorInformation() != null)
         {
            addForceSensors(simulationRobotDefinition, version.getSensorInformation());
            addPelvisImu(simulationRobotDefinition, version.getPhysicalProperties());
         }

         if (robotDefinitionMutator != null)
            robotDefinitionMutator.accept(simulationRobotDefinition);
      }

      return simulationRobotDefinition;
   }

   private void addForceSensors(RobotDefinition robotDefinition, Alice5SensorInformation sensorInformation)
   {
      SideDependentList<String> feetForceSensorNames = sensorInformation.getFeetForceSensorNames();
      SideDependentList<String> feetForceSensorParentJointNames = sensorInformation.getFeetForceSensorParentJointNames();

      for (RobotSide robotSide : RobotSide.values)
      {
         String forceSensorName = feetForceSensorNames.get(robotSide);
         String forceSensorParentJointName = feetForceSensorParentJointNames.get(robotSide);

         // wrench sensor at the ankle joint origin (MJCF l/r_leg_end site is directly below; sole frame handles the offset)
         RigidBodyTransform transform = new RigidBodyTransform();

         robotDefinition.getJointDefinition(forceSensorParentJointName).addSensorDefinition(new WrenchSensorDefinition(forceSensorName, transform));
      }
   }

   private void addPelvisImu(RobotDefinition robotDefinition, Alice5PhysicalProperties physicalProperties)
   {
      JointDefinition rootJoint = robotDefinition.getRootJointDefinitions().get(0);
      RigidBodyTransform imuTransform = new RigidBodyTransform();
      imuTransform.getTranslation().set(physicalProperties.getImuPositionInPelvis());
      rootJoint.addSensorDefinition(new IMUSensorDefinition(Alice5SensorInformation.PELVIS_IMU, imuTransform));
   }

   public RobotDefinition getControllerRobotDefinition()
   {
      if (controllerRobotDefinition == null)
      {
         controllerRobotDefinition = getSCS1RobotDefinition();
      }
      return controllerRobotDefinition;
   }
}
