package us.ihmc.alice5;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

/**
 * ALICE5 sensors: a single pelvis IMU (site "imu" in the MJCF) and per-foot wrench sensors
 * (synthesized in simulation; the real robot has 4 loadcells per foot — see docs/inventory.md).
 */
public class Alice5SensorInformation implements HumanoidRobotSensorInformation
{
   public static final String PELVIS_IMU = "pelvis_imu";

   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<>("LeftFootFTSensor", "RightFootFTSensor");
   private final SideDependentList<String> feetForceSensorParentJointNames = new SideDependentList<>("l_ankle_r", "r_ankle_r");

   public Alice5SensorInformation(Alice5VersionInterface version)
   {
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return new String[] {PELVIS_IMU};
   }

   @Override
   public AvatarRobotCameraParameters[] getCameraParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotCameraParameters getCameraParameters(int sensorId)
   {
      return null;
   }

   @Override
   public AvatarRobotLidarParameters[] getLidarParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotLidarParameters getLidarParameters(int sensorId)
   {
      return null;
   }

   @Override
   public AvatarRobotPointCloudParameters[] getPointCloudParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return null;
   }

   @Override
   public String[] getForceSensorNames()
   {
      return new String[0];
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   public SideDependentList<String> getFeetForceSensorParentJointNames()
   {
      return feetForceSensorParentJointNames;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return PELVIS_IMU;
   }

   @Override
   public ReferenceFrame getStereoCameraParentFrame(RobotSide side, CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getHeadFrame();
   }

   @Override
   public ReferenceFrame getExperimentalCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getHeadFrame();
   }

   @Override
   public RigidBodyTransform getExperimentalCameraTransform()
   {
      return new RigidBodyTransform();
   }

   @Override
   public RigidBodyTransform getStereoCameraTransform(RobotSide side)
   {
      return new RigidBodyTransform();
   }

   @Override
   public RigidBodyTransform getSteppingCameraTransform()
   {
      return new RigidBodyTransform();
   }
}
