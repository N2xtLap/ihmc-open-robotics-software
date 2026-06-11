package us.ihmc.alice5;

import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.alice5.parameters.model.HumanoidURDFParameterInterface;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Collection;

public interface Alice5VersionInterface extends RobotVersion
{
   Collection<String> getModelPath();

   Alice5JointMap getJointMap();

   Alice5SensorInformation getSensorInformation();

   Alice5PhysicalProperties getPhysicalProperties();

   HumanoidURDFParameterInterface getURDFParameters();

   default boolean hasCycloidForearms()
   {
      return false;
   }

   @Override
   default boolean hasArm(RobotSide robotSide)
   {
      return true;
   }

   @Override
   default boolean hasHead()
   {
      return true;
   }
}
