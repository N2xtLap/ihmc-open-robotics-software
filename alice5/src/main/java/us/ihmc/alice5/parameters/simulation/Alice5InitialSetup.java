package us.ihmc.alice5.parameters.simulation;

import us.ihmc.alice5.Alice5VersionInterface;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

/**
 * ALICE5 initial pose: same crouch as the stand-prep set points so the controller starts at its target.
 */
public class Alice5InitialSetup extends HumanoidRobotInitialSetup
{
   public Alice5InitialSetup(Alice5VersionInterface version, RobotDefinition robotDefinition, HumanoidJointNameMap jointMap)
   {
      super(jointMap);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJoint(robotSide, LegJointName.HIP_ROLL, robotSide.negateIfRightSide(0.02));
         setJoint(robotSide, LegJointName.HIP_YAW, 0.0);
         setJoint(robotSide, LegJointName.HIP_PITCH, -0.35);
         setJoint(robotSide, LegJointName.KNEE_PITCH, 0.7);
         setJoint(robotSide, LegJointName.ANKLE_PITCH, -0.35);
         setJoint(robotSide, LegJointName.ANKLE_ROLL, robotSide.negateIfRightSide(-0.02));

         setJoint(robotSide, ArmJointName.SHOULDER_PITCH, 0.2);
         setJoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(0.15));
         setJoint(robotSide, ArmJointName.ELBOW_PITCH, -0.6);
      }

      setJoint(SpineJointName.SPINE_YAW, 0.0);
      setJoint(SpineJointName.SPINE_PITCH, 0.0);
      setJoint(SpineJointName.SPINE_ROLL, 0.0);
      setJoint(NeckJointName.DISTAL_NECK_YAW, 0.0);
      setJoint(NeckJointName.DISTAL_NECK_PITCH, 0.1);

      setRootJointHeightSuchThatLowestSoleIsAtZero(robotDefinition);
   }
}
