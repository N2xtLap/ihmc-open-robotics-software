package us.ihmc.alice5.bridge;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * SensorReaderFactory for the ALICE5 shared-memory hardware path.
 * Consumed by AvatarEstimatorThreadFactory.getSensorReader(), which calls
 * build(...) with the estimator full robot model joints and sensor definitions.
 */
public class Alice5ShmSensorReaderFactory implements SensorReaderFactory
{
   private final SensorProcessingConfiguration sensorProcessingConfiguration;

   private Alice5ShmSensorReader sensorReader;

   public Alice5ShmSensorReaderFactory(SensorProcessingConfiguration sensorProcessingConfiguration)
   {
      this.sensorProcessingConfiguration = sensorProcessingConfiguration;
   }

   @Override
   public void build(FloatingJointBasics rootJoint,
                     IMUDefinition[] imuDefinitions,
                     ForceSensorDefinition[] forceSensorDefinitions,
                     JointDesiredOutputListBasics estimatorDesiredJointDataHolder,
                     YoRegistry parentRegistry)
   {
      sensorReader = new Alice5ShmSensorReader(rootJoint, imuDefinitions, forceSensorDefinitions, sensorProcessingConfiguration, parentRegistry);
   }

   @Override
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return sensorReader == null ? null : sensorReader.getStateEstimatorSensorDefinitions();
   }

   @Override
   public boolean useStateEstimator()
   {
      return true;
   }
}
