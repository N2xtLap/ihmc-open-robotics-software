package us.ihmc.alice5;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * ALICE5 joint name map. Joint/link names follow the MJCF (alice5_virtual.xml) naming verbatim.
 * Serial 23-DOF: 12 legs, 3 spine (pelvis -> waist_p -> waist_r -> waist_y = chest), 2 neck, 3x2 arms (no shoulder yaw).
 */
public class Alice5JointMap implements HumanoidJointNameMap
{
   public static final String PELVIS_NAME = "pelvis";
   public static final String CHEST_NAME = "waist_yaw_link";
   public static final String HEAD_NAME = "head_pitch_link";

   private final SideDependentList<String> footNames = new SideDependentList<>("left_ankle_roll", "right_ankle_roll");

   private final LinkedHashMap<String, ImmutablePair<RobotSide, ArmJointName>> armJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, NeckJointName> neckJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, SpineJointName> spineJointNames = new LinkedHashMap<>();
   private final LinkedHashMap<String, ImmutablePair<RobotSide, LegJointName>> legJointNames = new LinkedHashMap<>();

   private final EnumMap<NeckJointName, String> neckJointStrings = new EnumMap<>(NeckJointName.class);
   private final SideDependentList<EnumMap<LegJointName, String>> legJointStrings = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<ArmJointName, String>> armJointStrings = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private final EnumMap<SpineJointName, String> spineJointStrings = new EnumMap<>(SpineJointName.class);

   private final LegJointName[] legJoints = {LegJointName.HIP_ROLL, LegJointName.HIP_YAW, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH,
                                             LegJointName.ANKLE_PITCH, LegJointName.ANKLE_ROLL};
   private final SpineJointName[] spineJoints = {SpineJointName.SPINE_PITCH, SpineJointName.SPINE_ROLL, SpineJointName.SPINE_YAW};
   private final NeckJointName[] neckJoints = {NeckJointName.DISTAL_NECK_YAW, NeckJointName.DISTAL_NECK_PITCH};
   private final ArmJointName[] armJoints = {ArmJointName.SHOULDER_PITCH, ArmJointName.SHOULDER_ROLL, ArmJointName.ELBOW_PITCH};

   private final Alice5PhysicalProperties physicalProperties;
   private final LinkedHashMap<String, JointRole> jointRoles = new LinkedHashMap<>();
   private final String[] jointNamesBeforeFeet = new String[2];
   private final String[] jointNames;

   private final HashSet<String> lastSimulatedJoints = new HashSet<>();

   public Alice5JointMap(Alice5PhysicalProperties physicalProperties)
   {
      this.physicalProperties = physicalProperties;

      for (RobotSide side : RobotSide.values)
      {
         String p = side == RobotSide.LEFT ? "l_" : "r_";
         legJointNames.put(p + "hip_r", new ImmutablePair<>(side, LegJointName.HIP_ROLL));
         legJointNames.put(p + "hip_y", new ImmutablePair<>(side, LegJointName.HIP_YAW));
         legJointNames.put(p + "hip_p", new ImmutablePair<>(side, LegJointName.HIP_PITCH));
         legJointNames.put(p + "knee_p", new ImmutablePair<>(side, LegJointName.KNEE_PITCH));
         legJointNames.put(p + "ankle_p", new ImmutablePair<>(side, LegJointName.ANKLE_PITCH));
         legJointNames.put(p + "ankle_r", new ImmutablePair<>(side, LegJointName.ANKLE_ROLL));

         armJointNames.put(p + "sh_p", new ImmutablePair<>(side, ArmJointName.SHOULDER_PITCH));
         armJointNames.put(p + "sh_r", new ImmutablePair<>(side, ArmJointName.SHOULDER_ROLL));
         armJointNames.put(p + "el_p", new ImmutablePair<>(side, ArmJointName.ELBOW_PITCH));
      }

      spineJointNames.put("waist_p", SpineJointName.SPINE_PITCH);
      spineJointNames.put("waist_r", SpineJointName.SPINE_ROLL);
      spineJointNames.put("waist_y", SpineJointName.SPINE_YAW);

      neckJointNames.put("head_y", NeckJointName.DISTAL_NECK_YAW);
      neckJointNames.put("head_p", NeckJointName.DISTAL_NECK_PITCH);

      legJointNames.forEach((name, pair) -> {
         legJointStrings.get(pair.getLeft()).put(pair.getRight(), name);
         jointRoles.put(name, JointRole.LEG);
      });
      armJointNames.forEach((name, pair) -> {
         armJointStrings.get(pair.getLeft()).put(pair.getRight(), name);
         jointRoles.put(name, JointRole.ARM);
      });
      spineJointNames.forEach((name, joint) -> {
         spineJointStrings.put(joint, name);
         jointRoles.put(name, JointRole.SPINE);
      });
      neckJointNames.forEach((name, joint) -> {
         neckJointStrings.put(joint, name);
         jointRoles.put(name, JointRole.NECK);
      });

      jointNamesBeforeFeet[0] = getJointBeforeFootName(RobotSide.LEFT);
      jointNamesBeforeFeet[1] = getJointBeforeFootName(RobotSide.RIGHT);

      // URDF tree order (matches alice5_virtual.xml document order)
      List<String> ordered = new ArrayList<>(List.of("waist_p", "waist_r", "waist_y", "head_y", "head_p",
                                                     "l_sh_p", "l_sh_r", "l_el_p", "r_sh_p", "r_sh_r", "r_el_p",
                                                     "l_hip_r", "l_hip_y", "l_hip_p", "l_knee_p", "l_ankle_p", "l_ankle_r",
                                                     "r_hip_r", "r_hip_y", "r_hip_p", "r_knee_p", "r_ankle_p", "r_ankle_r"));
      jointNames = ordered.toArray(new String[0]);

      lastSimulatedJoints.add("l_el_p");
      lastSimulatedJoints.add("r_el_p");
   }

   @Override
   public ImmutablePair<RobotSide, ArmJointName> getArmJointName(String jointName)
   {
      return armJointNames.get(jointName);
   }

   @Override
   public SideDependentList<String> getNameOfJointBeforeHands()
   {
      return new SideDependentList<>("l_el_p", "r_el_p");
   }

   @Override
   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      return new RigidBodyTransform();
   }

   @Override
   public String getPelvisName()
   {
      return PELVIS_NAME;
   }

   @Override
   public String getChestName()
   {
      return CHEST_NAME;
   }

   @Override
   public String getNameOfJointBeforeChest()
   {
      return "waist_y";
   }

   @Override
   public String[] getOrderedJointNames()
   {
      return jointNames;
   }

   @Override
   public String getLegJointName(RobotSide robotSide, LegJointName legJointName)
   {
      return legJointStrings.get(robotSide).get(legJointName);
   }

   @Override
   public String getArmJointName(RobotSide robotSide, ArmJointName armJointName)
   {
      return armJointStrings.get(robotSide).get(armJointName);
   }

   @Override
   public String getNeckJointName(NeckJointName neckJointName)
   {
      return neckJointStrings.get(neckJointName);
   }

   @Override
   public String getSpineJointName(SpineJointName spineJointName)
   {
      return spineJointStrings.get(spineJointName);
   }

   @Override
   public String[] getPositionControlledJointsForSimulation()
   {
      List<String> allJoints = new ArrayList<>();
      allJoints.addAll(getLegJointNamesAsStrings());
      allJoints.addAll(getSpineJointNamesAsStrings());
      return allJoints.toArray(new String[allJoints.size()]);
   }

   @Override
   public String getHandName(RobotSide robotSide)
   {
      // SIM-EXT ARM-1: the distal arm link doubles as the "hand" body. Returning null here meant
      // WalkingHighLevelHumanoidController never created the arm RigidBodyControlManagers, leaving
      // sh_p/sh_r/el_p completely uncontrolled in the WBC QP (arms drifted to ~87 deg abduction).
      return robotSide == RobotSide.LEFT ? "left_elbow_pitch_link" : "right_elbow_pitch_link";
   }

   @Override
   public String getForearmName(RobotSide robotSide)
   {
      return null;
   }

   @Override
   public String getFootName(RobotSide robotSide)
   {
      return footNames.get(robotSide);
   }

   @Override
   public ImmutablePair<RobotSide, LegJointName> getLegJointName(String jointName)
   {
      return legJointNames.get(jointName);
   }

   @Override
   public String getJointBeforeFootName(RobotSide robotSide)
   {
      return legJointStrings.get(robotSide).get(LegJointName.ANKLE_ROLL);
   }

   @Override
   public RigidBodyTransform getSoleToParentFrameTransform(RobotSide robotSide)
   {
      return physicalProperties.getSoleToAnkleFrameTransforms().get(robotSide);
   }

   @Override
   public String getModelName()
   {
      return "alice5";
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public NeckJointName getNeckJointName(String jointName)
   {
      return neckJointNames.get(jointName);
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return spineJointNames.get(jointName);
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return PELVIS_NAME;
   }

   @Override
   public String getHeadName()
   {
      return HEAD_NAME;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      return lastSimulatedJoints;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public RobotSide getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String jointBeforeFootName = getJointBeforeFootName(robotSide);
         if (jointBeforeFootName != null && jointBeforeFootName.equals(jointNameBeforeEndEffector))
            return robotSide;

         if (jointNameBeforeEndEffector.startsWith(robotSide == RobotSide.LEFT ? "l_" : "r_"))
            return robotSide;
      }

      throw new IllegalArgumentException(jointNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }

   public boolean hasCycloidForearm(RobotSide side)
   {
      return false;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJoints;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return armJoints;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return spineJoints;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return neckJoints;
   }
}
