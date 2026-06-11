package us.ihmc.alice5.parameters.controller;

import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

/**
 * Foot sole contact: 4 corner points of the control polygon. The control polygon corners coincide
 * with the physical loadcell positions of ALICE5 (see assets properties: contactFrontX/BackX/Y),
 * since footLengthForControl = contactFrontX - contactBackX and footWidthForControl = 2*contactY.
 */
public class Alice5ContactPointParameters extends RobotContactPointParameters<RobotSide>
{
   public Alice5ContactPointParameters(HumanoidJointNameMap jointMap, Alice5PhysicalProperties physicalProperties)
   {
      super(jointMap,
            physicalProperties.getToeWidthForControl(),
            physicalProperties.getFootWidthForControl(),
            physicalProperties.getFootLengthForControl(),
            physicalProperties.getSoleToAnkleFrameTransforms());

      createDefaultFootContactPoints();
   }

   public Alice5ContactPointParameters(HumanoidJointNameMap jointMap, Alice5PhysicalProperties physicalProperties,
                                       FootContactPoints<RobotSide> footContactPoints)
   {
      super(jointMap,
            physicalProperties.getToeWidthForControl(),
            physicalProperties.getFootWidthForControl(),
            physicalProperties.getFootLengthForControl(),
            physicalProperties.getSoleToAnkleFrameTransforms());

      createFootContactPoints(footContactPoints);
   }

   public int getNumberOfContactableBodies()
   {
      return 2;
   }
}
