package us.ihmc.alice5.parameters.controller;

import us.ihmc.alice5.Alice5JointMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * ALICE5 stand-prep pose: slight crouch within joint limits. Sign conventions: hip_p negative =
 * leg forward, knee_p positive = flexion, ankle_p negative compensates knee flexion to keep the
 * sole flat. Joint ranges are loaded at runtime from the model assets (not inlined here).
 */
public class Alice5StandPrepSetPoints implements WholeBodySetpointParameters
{
   private final Map<String, Double> setPoints = new LinkedHashMap<>();

   public Alice5StandPrepSetPoints(Alice5JointMap jointMap)
   {
      setPoints.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);
      setPoints.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      setPoints.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);
      setPoints.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_PITCH), 0.1);
      setPoints.put(jointMap.getNeckJointName(NeckJointName.DISTAL_NECK_YAW), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_YAW), 0.0);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_ROLL), robotSide.negateIfRightSide(0.02));
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.35);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 0.7);
         // -0.45 instead of -0.35: leans the body ~5.7 deg forward about the ankles. At -0.35 the
         // model CoM is only ~18 mm (1.1 deg) ahead of the heel contact edge, and the joint-space
         // PD gravity sag alone tips the robot backward while standing in STAND_PREP.
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.45);
         setPoints.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL), robotSide.negateIfRightSide(-0.02));

         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.2);
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(0.15));
         setPoints.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), -0.6);
      }
   }

   @Override
   public double getSetpoint(String jointName)
   {
      if (setPoints.containsKey(jointName))
         return setPoints.get(jointName);
      else
         return 0.0;
   }
}
