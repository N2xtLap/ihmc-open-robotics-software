package us.ihmc.alice5;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.TreeSet;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * SIM-EXT DOF audit guard.
 *
 * <p>Regression test for the ARM-1 class of bug: a JointMap name getter returning {@code null}
 * caused {@code FullHumanoidRobotModel.getHand(side)} to resolve to {@code null}, so
 * {@code WalkingHighLevelHumanoidController} never created the arm {@code RigidBodyControlManager}s.
 * The arm joints (sh_p/sh_r/el_p) then sat in <b>no</b> task of the WBC QP and drifted to ~87 deg
 * abduction while standing/walking.
 *
 * <p>This test reconstructs exactly which 1-DOF joints the controller would place under a
 * {@code RigidBodyControlManager}, mirroring the body lookups and joint-path construction done in
 * {@code WalkingHighLevelHumanoidController} (chest from pelvis, head from chest, hands from chest,
 * feet from pelvis). It then asserts that <b>every</b> 1-DOF joint of the robot is covered by at
 * least one of those control paths. If any joint is left uncontrolled (category C), the test fails
 * and names the offender.
 *
 * <p>Coverage chains (ALICE5 23-DOF serial):
 * <ul>
 *   <li>pelvis -&gt; chest (waist_yaw_link): waist_p, waist_r, waist_y</li>
 *   <li>chest -&gt; head (head_pitch_link): head_y, head_p</li>
 *   <li>chest -&gt; hand (l/r_elbow_pitch_link): sh_p, sh_r, el_p per side</li>
 *   <li>pelvis -&gt; foot (l/r_ankle_roll): the 6 leg joints per side (walking controller)</li>
 * </ul>
 */
public class Alice5JointControlCoverageTest
{
   @Test
   public void testAllOneDoFJointsAreUnderSomeControlManager()
   {
      Alice5RobotModel robotModel = new Alice5RobotModel(Alice5Version.V1_FULL_ROBOT, RobotTarget.SCS);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      RigidBodyBasics chest = fullRobotModel.getChest();
      RigidBodyBasics head = fullRobotModel.getHead();

      // Mirror WalkingHighLevelHumanoidController body->manager wiring. A null body means the
      // controller would skip creating that manager, leaving its chain uncontrolled (the ARM-1 bug).
      Set<String> controlled = new LinkedHashSet<>();

      // chest manager (base = pelvis) -> spine chain
      if (chest != null)
         addJointPath(controlled, pelvis, chest);

      // head manager (base = chest) -> neck chain
      if (head != null && chest != null)
         addJointPath(controlled, chest, head);

      // hand managers (base = chest) -> arm chains
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         RigidBodyBasics handBaseBody = chest != null ? chest : pelvis;
         if (hand != null)
            addJointPath(controlled, handBaseBody, hand);
      }

      // walking controller -> leg chains (pelvis -> foot)
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         if (foot != null)
            addJointPath(controlled, pelvis, foot);
      }

      OneDoFJointBasics[] allJoints = fullRobotModel.getOneDoFJoints();
      assertTrue(allJoints.length > 0, "Robot model reported zero 1-DOF joints; model failed to load?");

      Set<String> uncontrolled = new TreeSet<>();
      for (OneDoFJointBasics joint : allJoints)
      {
         if (!controlled.contains(joint.getName()))
            uncontrolled.add(joint.getName());
      }

      if (!uncontrolled.isEmpty())
      {
         fail("Uncontrolled 1-DOF joint(s) found (category C - not in any RigidBodyControlManager "
              + "chain nor the walking leg chains). These joints would drift in the WBC QP exactly "
              + "like the ARM-1 arm bug. Offenders: " + uncontrolled + ". Total joints=" + allJoints.length
              + ", controlled=" + controlled.size() + ". Check the corresponding JointMap name getter "
              + "(getChestName / getHeadName / getHandName / getFootName) and the matching link name in "
              + "alice5_description/urdf/alice5.urdf.");
      }
   }

   private static void addJointPath(Set<String> controlled, RigidBodyBasics base, RigidBodyBasics body)
   {
      OneDoFJointBasics[] path = MultiBodySystemTools.createOneDoFJointPath(base, body);
      Arrays.stream(path).forEach(joint -> controlled.add(joint.getName()));
   }
}
