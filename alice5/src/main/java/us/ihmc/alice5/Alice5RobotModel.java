package us.ihmc.alice5;

import com.jme3.math.Transform;
import us.ihmc.alice5.parameters.controller.Alice5ContactPointParameters;
import us.ihmc.alice5.parameters.controller.Alice5HighLevelControllerParameters;
import us.ihmc.alice5.parameters.controller.Alice5ICPSplitFractionCalculatorParameters;
import us.ihmc.alice5.parameters.controller.Alice5StateEstimatorParameters;
import us.ihmc.alice5.parameters.controller.Alice5WalkingControllerParameters;
import us.ihmc.alice5.parameters.diagnostic.Alice5DiagnosticParameters;
import us.ihmc.alice5.parameters.model.Alice5KinematicsCollisionModel;
import us.ihmc.alice5.parameters.model.Alice5PhysicalProperties;
import us.ihmc.alice5.parameters.model.Alice5SimulationCollisionModel;
import us.ihmc.alice5.parameters.model.Alice5URDFParameters;
import us.ihmc.alice5.parameters.planning.Alice5FootstepPlannerParameters;
import us.ihmc.alice5.parameters.planning.Alice5LocomotionParameters;
import us.ihmc.alice5.parameters.planning.Alice5SwingPlannerParameters;
import us.ihmc.alice5.parameters.planning.Alice5VisibilityGraphParameters;
import us.ihmc.alice5.parameters.simulation.Alice5InitialSetup;
import us.ihmc.avatar.AvatarSimulatedHandControlThread;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.kinematicsSimulation.SimulatedHandKinematicController;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning.CoPTrajectoryParameters;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.LocomotionParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelWrapper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.io.InputStream;

public class Alice5RobotModel implements DRCRobotModel
{
   static final boolean ENFORCE_UNIQUE_REFERENCE_FRAMES = false;

   private static final double DEFAULT_SIMULATE_DT = 0.0001;
   private static final double DEFAULT_ESTIMATE_DT = 0.001;
   private static final double DEFAULT_CONTROL_DT = 0.003;
   private static final double DEFAULT_FEEDBACK_CONTROLLER_DT = 0.002;

   private double simulateDT = DEFAULT_SIMULATE_DT;
   private double estimatorDT = DEFAULT_ESTIMATE_DT;
   private double controllerDT = DEFAULT_CONTROL_DT;
   private double feedbackControllerDT = DEFAULT_FEEDBACK_CONTROLLER_DT;
   private double stepGeneratorDT = 10 * controllerDT;

   protected final Alice5PhysicalProperties physicalProperties;
   protected final WalkingControllerParameters walkingControllerParameters;
   private final HighLevelControllerParameters highLevelControllerParameters;
   private final Alice5SensorInformation sensorInformation;
   protected final Alice5JointMap jointMap;
   protected final RobotContactPointParameters<RobotSide> contactPointParameters;
   private final CoPTrajectoryParameters copTrajectoryParameters = new CoPTrajectoryParameters();
   private final Alice5DiagnosticParameters diagnosticParameters;
   private final StateEstimatorParameters stateEstimatorParameters;

   private final RobotDefinition scs1RobotDefinition;
   private final RobotDefinition controllerRobotDefinition;
   private final LogModelProvider logModelProvider;
   private final Alice5ModelFactory modelFactory;

   protected final RobotTarget robotTarget;
   protected final Alice5VersionInterface robotVersion;

   public Alice5RobotModel(Alice5VersionInterface robotVersion)
   {
      this(robotVersion, RobotTarget.SCS);
   }

   public Alice5RobotModel(Alice5VersionInterface robotVersion, RobotTarget robotTarget)
   {
      this(robotVersion, robotTarget, null,
           new Alice5ContactPointParameters(robotVersion.getJointMap(), robotVersion.getPhysicalProperties()));
   }

   public Alice5RobotModel(Alice5VersionInterface robotVersion,
                           RobotTarget robotTarget,
                           MaterialDefinition robotMaterial,
                           RobotContactPointParameters<RobotSide> contactPointParameters,
                           String... imusToIgnore)
   {
      this.robotVersion = robotVersion;
      this.robotTarget = robotTarget;
      this.contactPointParameters = contactPointParameters;

      jointMap = robotVersion.getJointMap();
      sensorInformation = robotVersion.getSensorInformation();
      physicalProperties = robotVersion.getPhysicalProperties();

      walkingControllerParameters = new Alice5WalkingControllerParameters(robotVersion, robotTarget, jointMap, physicalProperties, contactPointParameters);
      highLevelControllerParameters = new Alice5HighLevelControllerParameters(robotVersion, jointMap, robotTarget);
      diagnosticParameters = new Alice5DiagnosticParameters(robotTarget, jointMap, sensorInformation, highLevelControllerParameters);
      stateEstimatorParameters = new Alice5StateEstimatorParameters(getEstimatorDT(), robotTarget, sensorInformation, jointMap);

      modelFactory = new Alice5ModelFactory(robotVersion, jointMap, contactPointParameters, new Alice5RigidBodyMutator(physicalProperties, imusToIgnore));
      logModelProvider = modelFactory.createLogModelProvider();
      scs1RobotDefinition = modelFactory.getSCS1RobotDefinition();
      controllerRobotDefinition = modelFactory.getControllerRobotDefinition();

      if (robotMaterial != null)
      {
         RobotDefinitionTools.setRobotDefinitionMaterial(scs1RobotDefinition, robotMaterial);
         RobotDefinitionTools.setRobotDefinitionMaterial(controllerRobotDefinition, robotMaterial);
      }
   }

   @Override
   public Alice5VersionInterface getRobotVersion()
   {
      return robotVersion;
   }

   public Alice5PhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public RobotDefinition getRobotDefinition()
   {
      return controllerRobotDefinition;
   }

   @Override
   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParameters;
   }

   @Override
   public Alice5JointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public String toString()
   {
      return Alice5URDFParameters.URDF_MODEL_NAME;
   }

   @Override
   public HumanoidRobotInitialSetup getDefaultRobotInitialSetup()
   {
      return new Alice5InitialSetup(getRobotVersion(), getRobotDefinition(), getJointMap());
   }

   @Override
   public double[] getPresetArmConfiguration(RobotSide side, PresetArmConfiguration presetArmConfiguration)
   {
      return null;
   }

   @Override
   public RobotContactPointParameters<RobotSide> getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public HandModel getHandModel(RobotSide side)
   {
      return null;
   }

   @Override
   public Alice5SensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return createFullRobotModel(ENFORCE_UNIQUE_REFERENCE_FRAMES);
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel(boolean enforceUniqueReferenceFrames)
   {
      return new FullHumanoidRobotModelWrapper(controllerRobotDefinition, jointMap, enforceUniqueReferenceFrames);
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;
      return new HumanoidFloatingRootJointRobot(scs1RobotDefinition, jointMap, enableJointDamping, enableTorqueVelocityLimits);
   }

   @Override
   public double getSimulateDT()
   {
      return simulateDT;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public double getControllerDT()
   {
      return controllerDT;
   }

   @Override
   public double getFeedbackControllerDT()
   {
      return feedbackControllerDT;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return null;
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager(ROS2Node ros2Node)
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return logModelProvider;
   }

   @Override
   public DataServerSettings getLogSettings()
   {
      // SIM-EXT V1: port override so concurrent sims on one machine do not collide on 8008.
      DataServerSettings logSettings = new DataServerSettings(true);
      logSettings.setPort(Integer.getInteger("alice5.dataserver.port", DataServerSettings.DEFAULT_PORT));
      return logSettings;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Alice5";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }

   @Override
   public InputStream getWholeBodyControllerParametersFile()
   {
      return getClass().getResourceAsStream(getParameterResourceName());
   }

   public String getParameterResourceName()
   {
      return "/us/ihmc/alice5/parameters/controller.xml";
   }

   @Override
   public String getParameterFileName()
   {
      return getParameterResourceName();
   }

   @Override
   public CoPTrajectoryParameters getCoPTrajectoryParameters()
   {
      return copTrajectoryParameters;
   }

   @Override
   public RobotCollisionModel getSimulationRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks)
   {
      Alice5SimulationCollisionModel collisionModel = new Alice5SimulationCollisionModel(jointMap, physicalProperties);
      collisionModel.setCollidableHelper(helper, robotCollisionMask, environmentCollisionMasks);
      return collisionModel;
   }

   @Override
   public RobotCollisionModel getHumanoidRobotKinematicsCollisionModel()
   {
      return new Alice5KinematicsCollisionModel(getJointMap());
   }

   @Override
   public AvatarSimulatedHandControlThread createSimulatedHandController(RealtimeROS2Node realtimeROS2Node, boolean kinematicsSimulation)
   {
      return null;
   }

   @Override
   public SimulatedHandKinematicController createSimulatedHandKinematicController(FullHumanoidRobotModel fullHumanoidRobotModel,
                                                                                  RealtimeROS2Node realtimeROS2Node,
                                                                                  DoubleProvider controllerTime)
   {
      return null;
   }

   @Override
   public DiagnosticParameters getDiagnoticParameters()
   {
      return diagnosticParameters;
   }

   @Override
   public RobotLowLevelMessenger newRobotLowLevelMessenger(ROS2Node ros2Node)
   {
      return null;
   }

   @Override
   public LocomotionParameters getLocomotionParameters()
   {
      return new Alice5LocomotionParameters();
   }

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return new Alice5FootstepPlannerParameters();
   }

   @Override
   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters(String fileNameSuffix)
   {
      return new Alice5FootstepPlannerParameters(fileNameSuffix);
   }

   @Override
   public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   {
      return new AStarBodyPathPlannerParameters();
   }

   @Override
   public VisibilityGraphsParametersBasics getVisibilityGraphsParameters()
   {
      return new Alice5VisibilityGraphParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return new Alice5SwingPlannerParameters();
   }

   @Override
   public SwingPlannerParametersBasics getSwingPlannerParameters(String fileNameSuffix)
   {
      return new Alice5SwingPlannerParameters(fileNameSuffix);
   }

   @Override
   public SplitFractionCalculatorParametersReadOnly getSplitFractionCalculatorParameters()
   {
      return new Alice5ICPSplitFractionCalculatorParameters();
   }

   @Override
   public double getStepGeneratorDT()
   {
      return stepGeneratorDT;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide robotSide)
   {
      return new Transform();
   }

   @Override
   public RigidBodyTransform getHandGraphicToHandFrameTransform(RobotSide side)
   {
      return new RigidBodyTransform();
   }

   public void setControllerDT(double controllerDT)
   {
      this.controllerDT = controllerDT;
   }

   public void setFeedbackControllerDT(double feedbackControllerDT)
   {
      this.feedbackControllerDT = feedbackControllerDT;
   }
}
