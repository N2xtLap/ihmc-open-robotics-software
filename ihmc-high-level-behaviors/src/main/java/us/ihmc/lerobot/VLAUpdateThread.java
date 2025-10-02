package us.ihmc.lerobot;

import behavior_msgs.msg.dds.VLAOperationMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Size;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.openpi.OpenpiClient;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.concurrent.CompletableFuture;

/**
 * Autonomy process thread for managing vision-language-action (VLA) inference and supporting remote UI.
 * Manages communication with the Python side, which is running the openpi.
 */
public class VLAUpdateThread extends RepeatingTaskThread
{
   public static final ROS2IOTopicPair<VLAOperationMessage> UI = new ROS2IOTopicPair<>(new ROS2Topic<>().withPrefix("vla_ui")
                                                                                                        .withTypeName(VLAOperationMessage.class));
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImageSensor zedSensor;
   private String status = "Not connected to openpi";
   private final FrequencyCalculator statusFrequency = new FrequencyCalculator();
   private final FramePose3D framePose = new FramePose3D();
   private final SideDependentList<Pose3D> stateHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> stateForearmPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> actionHandPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private final SideDependentList<Pose3D> actionForearmPoses = new SideDependentList<>(new Pose3D(), new Pose3D());
   private long actionTimestampNanos = 0L;
   private long numberOfActionsReceived = 0L;
   private long numberOfActionsTaken = 0L;
   private final OpenpiClient openpiClient = new OpenpiClient("10.6.192.65");
   private CompletableFuture<byte[]> openpiRequest;
   private final Notification requested = new Notification();
   private final Deque<DoubleBuffer> actionPlan = new ArrayDeque<>();
   private final Throttler actionThrottler = new Throttler().setFrequency(5.0);
   private final int planSize = 5;
   private final SideDependentList<Mat> images = new SideDependentList<>(new Mat(224, 224, opencv_core.CV_8UC3), new Mat(224, 224, opencv_core.CV_8UC3));

   private final LatestTimestampModifiable latestTimestampModifiable;
   private long sequenceID = 0L;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<VLAOperationMessage> uiCommandSubscription;
   private final ROS2Publisher<VLAOperationMessage> uiStatusPublisher;

   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kstInputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> kstStatePublisher;

   public VLAUpdateThread(ROS2Node ros2Node,
                          ROS2PeerClockOffsetEstimator clockOffsetEstimator,
                          DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          ImageSensor zedSensor)
   {
      super(VLAUpdateThread.class.getSimpleName());

      this.syncedRobot = syncedRobot;
      this.zedSensor = zedSensor;

      setFrequencyLimit(30.0);

      actionHandPoses.forEach(Pose3D::setToNaN);
      actionForearmPoses.forEach(Pose3D::setToNaN);

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator));
      latestTimestampModifiable.modify(); // On startup, we want the initial state to propagate
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      uiCommandSubscription = ROS2Tools.createNotificationSubscription(ros2Node, UI.getTopic(ROS2ActorDesignation.ROBOT.getIncomingQualifier()));
      uiStatusPublisher = ros2Node.createPublisher(UI.getTopic(ROS2ActorDesignation.ROBOT.getOutgoingQualifier()));

      kstInputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotModel.getSimpleRobotName()));
      kstStatePublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingStateTopic(robotModel.getSimpleRobotName()));
   }

   @Override
   public void runTask()
   {
      if (uiCommandSubscription.poll())
      {
         VLAOperationMessage uiCommand = uiCommandSubscription.read();
         latestTimestampModifiable.fromMessage(uiCommand.getLatestTimestampModifiable());
         boolean wasRunning = running.getValue();
         running.fromMessage(uiCommand.getRunning());
         if (!wasRunning && running.getValue())
         {
            ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
            toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
            kstStatePublisher.publish(toolboxStateMessage);
         }
         controlRobot.fromMessage(uiCommand.getControlRobot());
      }

      if (running.getValue())
      {
         if (openpiRequest == null)
         {
            if (actionPlan.isEmpty())
            {
               boolean requestValid = true;

               ByteBuffer state = openpiClient.getState();
               state.clear();
               synchronized (syncedRobot)
               {
                  requestValid &= syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.02);
                  if (requestValid)
                  {
                     for (RobotSide side : RobotSide.values)
                     {
                        Pose3D stateHandPose = stateHandPoses.get(side);
                        framePose.setToZero(syncedRobot.getFullRobotModel().getHand(side).getParentJoint().getFrameAfterJoint());
                        framePose.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
                        stateHandPose.set(framePose);
                        state.putFloat(stateHandPose.getPosition().getX32());
                        state.putFloat(stateHandPose.getPosition().getY32());
                        state.putFloat(stateHandPose.getPosition().getZ32());
                        state.putFloat(stateHandPose.getOrientation().getX32());
                        state.putFloat(stateHandPose.getOrientation().getY32());
                        state.putFloat(stateHandPose.getOrientation().getZ32());
                        state.putFloat(stateHandPose.getOrientation().getS32());
                        Pose3D stateForearmPose = stateForearmPoses.get(side);
                        framePose.setToZero(syncedRobot.getFullRobotModel().getForearm(side).getParentJoint().getFrameAfterJoint());
                        framePose.changeFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
                        stateForearmPose.set(framePose);
                        state.putFloat(stateForearmPose.getPosition().getX32());
                        state.putFloat(stateForearmPose.getPosition().getY32());
                        state.putFloat(stateForearmPose.getPosition().getZ32());
                        state.putFloat(stateForearmPose.getOrientation().getX32());
                        state.putFloat(stateForearmPose.getOrientation().getY32());
                        state.putFloat(stateForearmPose.getOrientation().getZ32());
                        state.putFloat(stateForearmPose.getOrientation().getS32());
                     }
                  }
               }

               for (RobotSide side : RobotSide.values)
               {
                  RawImage image = zedSensor.getImage(zedSensor.getImageKeys()[side.ordinal()]);

                  requestValid &= image != null;
                  if (requestValid)
                  {
                     Mat rgbColor = new Mat();
                     image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), rgbColor, PixelFormat.RGB8);
                     image.release();

                     Size cropSize = new Size(224, 224); // Square frame for siglip
                     int scaleWidth = image.getWidth() * cropSize.height() / image.getHeight(); // Account for aspect ratio
                     Size scaleDownSize = new Size(scaleWidth, cropSize.height());
                     Mat resized = new Mat(scaleDownSize, opencv_core.CV_8UC3);
                     opencv_imgproc.resize(rgbColor, resized, scaleDownSize);
                     scaleDownSize.close();
                     rgbColor.release();

                     Point cropOffset = new Point((resized.cols() - cropSize.width()) / 2, 0); // Center crop horizontally
                     Rect roi = new Rect(cropOffset, cropSize);
                     Mat cropped = new Mat(resized, roi);
                     cropped.copyTo(images.get(side));
                     resized.close();
                     cropSize.close();
                     cropOffset.close();
                     roi.close();

                     cropped.data().get(openpiClient.getImages().get(side).array());
                     cropped.close();
                  }
               }

               if (requestValid)
               {
                  openpiRequest = openpiClient.request();
                  if (openpiRequest == null)
                     status = "Could not connect to server at ws://" + openpiClient.getHost() + ":" + openpiClient.getPort();
                  else
                  {
                     requested.set();
                     status = "Requested inference...";
                  }
               }
               else
               {
                  status = "Waiting for robot data...";
               }
            }
         }
         else if (openpiRequest.isDone())
         {
            if (!openpiRequest.isCompletedExceptionally())
            {
               openpiClient.unpack(openpiRequest);

               DoubleBuffer actionChunk = openpiClient.getActionChunk().asDoubleBuffer();
               for (int i = 0; i < planSize; i++)
               {
                  DoubleBuffer action = DoubleBuffer.allocate(28);
                  action.put(0, actionChunk, i * 28, 28);
                  actionPlan.addLast(action);
               }
               numberOfActionsReceived += actionPlan.size();
            }

            openpiRequest = null;
         }

         if (actionThrottler.run())
         {
            DoubleBuffer action = actionPlan.pollFirst();
            if (action != null)
            {
               actionTimestampNanos = System.nanoTime(); // TODO: Get this from the policy on the python side?
               ++numberOfActionsTaken;
               status = "Taking action %d".formatted(numberOfActionsTaken);

               synchronized (syncedRobot)
               {
                  int i = 0;
                  for (RobotSide side : RobotSide.values)
                  {
                     framePose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
                     framePose.getPosition().set(action.get(i++), action.get(i++), action.get(i++));
                     framePose.getOrientation().set(action.get(i++), action.get(i++), action.get(i++), action.get(i++));
                     framePose.changeFrame(ReferenceFrame.getWorldFrame());
                     actionHandPoses.get(side).set(framePose);
                     framePose.setToZero(syncedRobot.getReferenceFrames().getPelvisFrame());
                     framePose.getPosition().set(action.get(i++), action.get(i++), action.get(i++));
                     framePose.getOrientation().set(action.get(i++), action.get(i++), action.get(i++), action.get(i++));
                     framePose.changeFrame(ReferenceFrame.getWorldFrame());
                     actionForearmPoses.get(side).set(framePose);
                  }
               }

               KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();
               ikInputMessage.setStreamToController(controlRobot.getValue());
               ikInputMessage.setTimestamp(actionTimestampNanos);
               for (RobotSide side : RobotSide.values)
               {
                  KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
                  rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getHand(side).hashCode());
                  rigidBodyMessage.getDesiredPositionInWorld().set(actionHandPoses.get(side).getTranslation());
                  rigidBodyMessage.getDesiredOrientationInWorld().set(actionHandPoses.get(side).getRotation());
                  rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.02);
                  rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.02);
                  rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.02);
                  ikInputMessage.getInputs().add().set(rigidBodyMessage);

                  rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
                  rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getForearm(side).hashCode());
                  rigidBodyMessage.getDesiredPositionInWorld().set(actionForearmPoses.get(side).getTranslation());
                  rigidBodyMessage.getLinearSelectionMatrix().setXSelected(false); // Disable position tracking for forearm
                  rigidBodyMessage.getLinearSelectionMatrix().setYSelected(false);
                  rigidBodyMessage.getLinearSelectionMatrix().setZSelected(false);
                  rigidBodyMessage.getDesiredOrientationInWorld().set(actionForearmPoses.get(side).getRotation());
                  rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.01);
                  rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.01);
                  rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.001);
                  ikInputMessage.getInputs().add().set(rigidBodyMessage);
               }
               kstInputPublisher.publish(ikInputMessage);
            }
         }
      }
      else
      {
         actionPlan.clear();
         status = "Not running";
      }

      VLAOperationMessage uiStatus = new VLAOperationMessage();
      latestTimestampModifiable.toMessage(uiStatus.getLatestTimestampModifiable());
      uiStatus.setSequenceId(sequenceID++);
      uiStatus.setRunning(running.toMessage());
      uiStatus.setControlRobot(controlRobot.toMessage());
      for (RobotSide side : RobotSide.values)
      {
         uiStatus.getActionHandPoses()[side.ordinal()].set(actionHandPoses.get(side));
         uiStatus.getActionForearmPoses()[side.ordinal()].set(actionForearmPoses.get(side));
      }
      uiStatus.setStatusMessage("%-30s Actions: %d".formatted(status, numberOfActionsReceived));
      uiStatusPublisher.publish(uiStatus);
   }

   public void destroy()
   {
      blockingKill();
   }

   public Notification getRequested()
   {
      return requested;
   }

   public SideDependentList<Mat> getImages()
   {
      return images;
   }
}
