package us.ihmc.alice5.bridge;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.SensorProcessingConfiguration;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Hardware-path sensor reader for the ALICE5 shared-memory bridge.
 * <p>
 * The SensorDataContext is populated by {@link Alice5ShmCommunication#read(SensorDataContext)}
 * on the master thread; this class only consumes the context in
 * {@link #compute(long, SensorDataContext)} and feeds it into {@link SensorProcessing}
 * (same pattern as the non-perfect branch of SCS2SensorReader).
 */
public class Alice5ShmSensorReader implements SensorReader
{
   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();
   private final List<OneDoFJointBasics> oneDoFJoints = new ArrayList<>();
   private final List<IMUDefinition> imuDefinitions = new ArrayList<>();
   private final List<ForceSensorDefinition> forceSensorDefinitions = new ArrayList<>();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final SensorProcessing sensorProcessing;

   public Alice5ShmSensorReader(FloatingJointBasics rootJoint,
                                IMUDefinition[] imuDefinitions,
                                ForceSensorDefinition[] forceSensorDefinitions,
                                SensorProcessingConfiguration sensorProcessingConfiguration,
                                YoRegistry parentRegistry)
   {
      for (JointBasics joint : rootJoint.subtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;
            oneDoFJoints.add(oneDoFJoint);
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      if (imuDefinitions != null)
      {
         for (IMUDefinition imuDefinition : imuDefinitions)
         {
            this.imuDefinitions.add(imuDefinition);
            stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
         }
      }

      if (forceSensorDefinitions != null)
      {
         for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
         {
            this.forceSensorDefinitions.add(forceSensorDefinition);
            stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
         }
      }

      sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorProcessingConfiguration, registry);
      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      sensorProcessing.initialize();
   }

   @Override
   public long read(SensorDataContext sensorDataContext)
   {
      // The hardware communication interface fills the context on the master thread;
      // this method is not used on the hardware path.
      return System.nanoTime();
   }

   @Override
   public void compute(long timestamp, SensorDataContext sensorDataContext)
   {
      if (oneDoFJoints.isEmpty() || !sensorDataContext.isJointRegistered(oneDoFJoints.get(0).getName()))
         return; // Context not populated yet.

      for (int i = 0; i < oneDoFJoints.size(); i++)
      {
         OneDoFJointBasics joint = oneDoFJoints.get(i);
         LowLevelState jointState = sensorDataContext.getMeasuredJointState(joint.getName());
         sensorProcessing.setJointPositionSensorValue(joint, jointState.getPosition());
         sensorProcessing.setJointVelocitySensorValue(joint, jointState.getVelocity());
         sensorProcessing.setJointTauSensorValue(joint, jointState.getEffort());
      }

      for (int i = 0; i < imuDefinitions.size(); i++)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(i);
         ImuData imuData = sensorDataContext.getImuMeasurement(imuDefinition.getName());
         sensorProcessing.setOrientationSensorValue(imuDefinition, imuData.getOrientation());
         sensorProcessing.setAngularVelocitySensorValue(imuDefinition, imuData.getAngularVelocity());
         sensorProcessing.setLinearAccelerationSensorValue(imuDefinition, imuData.getLinearAcceleration());
      }

      for (int i = 0; i < forceSensorDefinitions.size(); i++)
      {
         ForceSensorDefinition forceSensorDefinition = forceSensorDefinitions.get(i);
         sensorProcessing.setForceSensorValue(forceSensorDefinition, sensorDataContext.getForceSensorMeasurement(forceSensorDefinition.getSensorName()));
      }

      sensorProcessing.startComputation(timestamp, timestamp, timestamp);
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      return sensorProcessing;
   }

   @Override
   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      return sensorProcessing.getRawSensorOutputMap();
   }

   @Override
   public SensorProcessing getSensorProcessing()
   {
      return sensorProcessing;
   }
}
