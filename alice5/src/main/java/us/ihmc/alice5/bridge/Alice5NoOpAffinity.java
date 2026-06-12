package us.ihmc.alice5.bridge;

import us.ihmc.affinity.Processor;
import us.ihmc.avatar.wholeBodyHardwareControl.AvatarAffinityInterface;
import us.ihmc.realtime.PriorityParameters;

/**
 * No-op affinity for non-realtime runs (useRealtimeThreads = false): none of the
 * processor/priority assignments are queried in that mode.
 */
public class Alice5NoOpAffinity implements AvatarAffinityInterface
{
   @Override
   public Processor getMasterThreadProcessor()
   {
      return null;
   }

   @Override
   public Processor getEstimatorThreadProcessor()
   {
      return null;
   }

   @Override
   public Processor getControllerThreadProcessor()
   {
      return null;
   }

   @Override
   public Processor getStepGeneratorThreadProcessor()
   {
      return null;
   }

   @Override
   public Processor getIKStreamingThreadProcessor()
   {
      return null;
   }

   @Override
   public PriorityParameters getMasterThreadPriority()
   {
      return null;
   }

   @Override
   public PriorityParameters getEstimatorThreadPriority()
   {
      return null;
   }

   @Override
   public PriorityParameters getControllerThreadPriority()
   {
      return null;
   }

   @Override
   public PriorityParameters getStepGeneratorThreadPriority()
   {
      return null;
   }

   @Override
   public PriorityParameters getIKStreamingThreadPriority()
   {
      return null;
   }
}
