package us.ihmc.alice5.bridge;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.concurrent.atomic.AtomicIntegerArray;
import java.util.concurrent.atomic.AtomicLong;

import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * SIM-EXT RT-OBS: real-time control health diagnostics (Java writer side).
 * <p>
 * Maintains the Java region of the diagnostics block /dev/shm/alice5_wbc_stats
 * (4096 B, header + Rust region + Java region; layout frozen in
 * framework/crates/wbc-bridge/src/stats.rs) and publishes an "rtHealth" YoRegistry
 * with tick/GC/jitter statistics plus alarm booleans.
 * <p>
 * Strictly best-effort: if the stats block cannot be opened or is corrupt, the
 * shared-memory part is silently disabled (process-local tick/GC stats and the
 * YoVariables keep working). Nothing here may affect the control path -- all heavy work
 * happens in the 1 Hz {@link #updateAndSummarize(boolean)} call from the status
 * logger thread; the controller-thread hooks only stamp timestamps and bump a
 * fixed histogram bucket.
 */
public class Alice5RtStats
{
   public static final String STATS_PATH = "/dev/shm/alice5_wbc_stats";
   public static final int STATS_SIZE = 4096;
   public static final long STATS_MAGIC = 0x3153424F54523541L; // "A5RTOBS1" little-endian
   public static final int STATS_VERSION = 1;
   public static final int RUST_OFFSET = 64;
   public static final int RUST_SIZE = 128;
   public static final int JAVA_OFFSET = 1024;
   public static final int JAVA_SIZE = 128;

   // Header field offsets.
   private static final int HDR_MAGIC = 0;
   private static final int HDR_VERSION = 8;
   private static final int HDR_RUST_OFFSET = 12;
   private static final int HDR_RUST_SIZE = 16;
   private static final int HDR_JAVA_OFFSET = 20;
   private static final int HDR_JAVA_SIZE = 24;

   // Rust region field offsets (bytes from RUST_OFFSET); Rust/twin is the writer.
   private static final int R_SEQ = 0;
   private static final int R_HEARTBEAT = 8;
   private static final int R_T_MONO_NS = 16;
   private static final int R_JITTER_P50_US = 24;
   private static final int R_JITTER_P99_US = 28;
   private static final int R_OVERRUN_COUNT = 32;
   private static final int R_STALENESS_EVENTS = 40;
   private static final int R_FSM_TRANSITIONS = 48;
   private static final int R_CYCLE = 56;

   // Java region field offsets (bytes from JAVA_OFFSET); this class is the writer.
   private static final int J_SEQ = 0;
   private static final int J_HEARTBEAT = 8;
   private static final int J_T_NS = 16;
   private static final int J_TICK_P50_US = 24;
   private static final int J_TICK_P99_US = 28;
   private static final int J_DEADLINE_MISS_COUNT = 32;
   private static final int J_GC_LAST_PAUSE_MS = 40;
   private static final int J_GC_TOTAL_PAUSE_MS = 48;
   private static final int J_FREEZE_COUNT = 56;

   private static final int SEQLOCK_MAX_RETRIES = 8;
   private static final VarHandle INT_VIEW = MethodHandles.byteBufferViewVarHandle(int[].class, ByteOrder.LITTLE_ENDIAN);

   // Alarm thresholds.
   private static final double GC_PAUSE_ALARM_MS = 2.0;
   private static final long JITTER_ALARM_US = 200L;

   // Controller tick histogram: 1024 buckets x 10 us (0..10.24 ms, last bucket = overflow).
   private static final int TICK_BUCKET_US = 10;
   private static final int TICK_N_BUCKETS = 1024;

   private final YoRegistry registry = new YoRegistry("rtHealth");

   private final YoBoolean statsBlockOk = new YoBoolean("statsBlockOk", registry);
   private final YoDouble tickP50Ms = new YoDouble("tickP50Ms", registry);
   private final YoDouble tickP99Ms = new YoDouble("tickP99Ms", registry);
   private final YoLong deadlineMissCount = new YoLong("deadlineMissCount", registry);
   private final YoDouble gcLastPauseMs = new YoDouble("gcLastPauseMs", registry);
   private final YoDouble gcCumulativePauseMs = new YoDouble("gcCumulativePauseMs", registry);
   private final YoLong freezeCount = new YoLong("freezeCount", registry);
   private final YoLong jitterP50Us = new YoLong("jitterP50Us", registry);
   private final YoLong jitterP99Us = new YoLong("jitterP99Us", registry);
   private final YoLong rustOverrunCount = new YoLong("rustOverrunCount", registry);
   private final YoLong stalenessEvents = new YoLong("stalenessEvents", registry);
   private final YoLong fsmTransitions = new YoLong("fsmTransitions", registry);
   private final YoBoolean alarmTickOverrun = new YoBoolean("alarmTickOverrun", registry);
   private final YoBoolean alarmGc = new YoBoolean("alarmGc", registry);
   private final YoBoolean alarmJitter = new YoBoolean("alarmJitter", registry);
   private final YoBoolean alarmStaleness = new YoBoolean("alarmStaleness", registry);

   private final long controlDTNs;
   // Stop-the-world accounting: concurrent collectors (ZGC/Shenandoah) split their MXBeans into
   // "...Pauses" (STW time) and "...Cycles" (whole concurrent cycle). Counting cycle time as
   // pause would permanently trip the GC alarm, so prefer the "Pauses" beans when present;
   // G1/Parallel have no such split and use all beans (their collection time IS pause time).
   private final List<GarbageCollectorMXBean> gcBeans = selectPauseBeans(ManagementFactory.getGarbageCollectorMXBeans());

   // Controller-thread side (hot path: one histogram increment, no allocation).
   private final AtomicIntegerArray tickHistogram = new AtomicIntegerArray(TICK_N_BUCKETS);
   private final AtomicLong tickSamples = new AtomicLong(0);
   private final AtomicLong deadlineMisses = new AtomicLong(0);
   // ControllerTask's ThreadTimer "ControllerTimer" YoDouble (ms). The post-task callback runs
   // before ThreadTimer.stop(), so the value read there is the duration of the PREVIOUS tick --
   // each tick is still sampled exactly once, just shifted by one tick (fine for p50/p99).
   private YoDouble controllerTickTimerMs = null;

   // 1 Hz sampler state (status logger thread only).
   private final int[] histSnapshot = new int[TICK_N_BUCKETS];
   private long gcPrevTimeMs = 0;
   private long gcPrevCount = 0;
   private double gcCumMs = 0.0;
   private boolean prevFrozen = false;
   private long freezes = 0;
   private long prevStalenessEvents = -1;
   private boolean tickOverrunPrev = false;
   private long javaHeartbeat = 0;
   private long javaSeq = 0;

   private FileChannel channel;
   private MappedByteBuffer buffer; // null = shm disabled (process-local stats keep working)

   private final StringBuilder summaryBuilder = new StringBuilder(160);

   public Alice5RtStats(double controlDT)
   {
      this.controlDTNs = (long) (controlDT * 1.0e9);

      String error = tryOpenStatsBlock();
      if (error != null)
      {
         LogTools.warn("RT stats block unavailable ({}) -- shared diagnostics disabled, control unaffected.", error);
         closeQuietly();
      }
      statsBlockOk.set(buffer != null);

      long timeMs = 0;
      long count = 0;
      for (int i = 0; i < gcBeans.size(); i++)
      {
         timeMs += Math.max(0, gcBeans.get(i).getCollectionTime());
         count += Math.max(0, gcBeans.get(i).getCollectionCount());
      }
      gcPrevTimeMs = timeMs;
      gcPrevCount = count;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   /** @return null on success, otherwise the reason the block is unusable. */
   private String tryOpenStatsBlock()
   {
      try
      {
         channel = FileChannel.open(Path.of(STATS_PATH), StandardOpenOption.READ, StandardOpenOption.WRITE, StandardOpenOption.CREATE);
         if (channel.size() > STATS_SIZE)
            return "unexpected size " + channel.size();
         MappedByteBuffer mapped = channel.map(FileChannel.MapMode.READ_WRITE, 0, STATS_SIZE);
         mapped.order(ByteOrder.LITTLE_ENDIAN);

         long magic = mapped.getLong(HDR_MAGIC);
         if (magic == STATS_MAGIC)
         {
            if (mapped.getInt(HDR_VERSION) != STATS_VERSION)
               return "version mismatch: " + mapped.getInt(HDR_VERSION);
            if (mapped.getInt(HDR_RUST_OFFSET) != RUST_OFFSET || mapped.getInt(HDR_JAVA_OFFSET) != JAVA_OFFSET)
               return "region offsets mismatch";
         }
         else if (magic == 0L)
         {
            // Fresh block: initialize header (idempotent constants; benign creation race).
            mapped.putInt(HDR_VERSION, STATS_VERSION);
            mapped.putInt(HDR_RUST_OFFSET, RUST_OFFSET);
            mapped.putInt(HDR_RUST_SIZE, RUST_SIZE);
            mapped.putInt(HDR_JAVA_OFFSET, JAVA_OFFSET);
            mapped.putInt(HDR_JAVA_SIZE, JAVA_SIZE);
            VarHandle.releaseFence();
            mapped.putLong(HDR_MAGIC, STATS_MAGIC);
         }
         else
         {
            return String.format("magic corrupt: 0x%016X", magic);
         }

         buffer = mapped;
         return null;
      }
      catch (IOException e)
      {
         return "I/O error: " + e.getMessage();
      }
   }

   private void closeQuietly()
   {
      buffer = null;
      try
      {
         if (channel != null)
            channel.close();
      }
      catch (IOException e)
      {
         // Best effort.
      }
      channel = null;
   }

   /** Wire the upstream tick duration source ("ControllerTimer" YoDouble, ms). May be null. */
   public void setControllerTickTimer(YoDouble controllerTimerMs)
   {
      this.controllerTickTimerMs = controllerTimerMs;
      if (controllerTimerMs == null)
         LogTools.warn("ControllerTimer variable not found -- controller tick stats disabled.");
   }

   /** Controller-thread post-tick hook (hot path: histogram bump, no allocation). */
   public void onControllerTickEnd()
   {
      YoDouble timer = controllerTickTimerMs;
      if (timer == null)
         return;
      double ms = timer.getValue();
      if (ms <= 0.0)
         return; // first tick: previous duration not measured yet
      long durationNs = (long) (ms * 1.0e6);
      int bucket = (int) (durationNs / 1000L / TICK_BUCKET_US);
      if (bucket >= TICK_N_BUCKETS)
         bucket = TICK_N_BUCKETS - 1;
      tickHistogram.incrementAndGet(bucket);
      tickSamples.incrementAndGet();
      if (durationNs > controlDTNs)
         deadlineMisses.incrementAndGet();
   }

   /**
    * 1 Hz update from the status logger thread: computes tick percentiles over the last
    * window, samples GC and freeze state, reads the Rust region, refreshes the YoVariables
    * and alarms, writes the Java region, and returns a compact log token for the RUNTIME line.
    */
   public String updateAndSummarize(boolean frozenNow)
   {
      // Controller tick percentiles over the last window (snapshot-and-reset).
      long samples = 0;
      for (int i = 0; i < TICK_N_BUCKETS; i++)
      {
         histSnapshot[i] = tickHistogram.getAndSet(i, 0);
         samples += histSnapshot[i];
      }
      tickSamples.addAndGet(-samples);
      long tickP50UsValue = percentileUs(histSnapshot, samples, 0.50);
      long tickP99UsValue = percentileUs(histSnapshot, samples, 0.99);
      tickP50Ms.set(tickP50UsValue / 1000.0);
      tickP99Ms.set(tickP99UsValue / 1000.0);
      deadlineMissCount.set(deadlineMisses.get());

      // GC deltas (cumulative collection time across all collectors).
      long timeMs = 0;
      long count = 0;
      for (int i = 0; i < gcBeans.size(); i++)
      {
         timeMs += Math.max(0, gcBeans.get(i).getCollectionTime());
         count += Math.max(0, gcBeans.get(i).getCollectionCount());
      }
      long deltaTime = timeMs - gcPrevTimeMs;
      long deltaCount = count - gcPrevCount;
      gcPrevTimeMs = timeMs;
      gcPrevCount = count;
      double lastPause = deltaCount > 0 ? (double) deltaTime / deltaCount : 0.0;
      gcCumMs += Math.max(0, deltaTime);
      gcLastPauseMs.set(lastPause);
      gcCumulativePauseMs.set(gcCumMs);

      // Freeze rising edges.
      if (frozenNow && !prevFrozen)
         freezes++;
      prevFrozen = frozenNow;
      freezeCount.set(freezes);

      // Rust region (robot/twin side health), seqlock read.
      long jitP50 = 0, jitP99 = 0, overruns = 0, staleEvents = 0, transitions = 0;
      boolean rustOk = false;
      if (buffer != null)
      {
         for (int attempt = 0; attempt < SEQLOCK_MAX_RETRIES; attempt++)
         {
            int s1 = (int) INT_VIEW.getVolatile(buffer, RUST_OFFSET + R_SEQ);
            if ((s1 & 1) != 0)
               continue;
            jitP50 = buffer.getInt(RUST_OFFSET + R_JITTER_P50_US) & 0xFFFFFFFFL;
            jitP99 = buffer.getInt(RUST_OFFSET + R_JITTER_P99_US) & 0xFFFFFFFFL;
            overruns = buffer.getLong(RUST_OFFSET + R_OVERRUN_COUNT);
            staleEvents = buffer.getLong(RUST_OFFSET + R_STALENESS_EVENTS);
            transitions = buffer.getLong(RUST_OFFSET + R_FSM_TRANSITIONS);
            VarHandle.acquireFence();
            int s2 = (int) INT_VIEW.getVolatile(buffer, RUST_OFFSET + R_SEQ);
            if (s1 == s2)
            {
               rustOk = true;
               break;
            }
         }
      }
      if (rustOk)
      {
         jitterP50Us.set(jitP50);
         jitterP99Us.set(jitP99);
         rustOverrunCount.set(overruns);
         stalenessEvents.set(staleEvents);
         fsmTransitions.set(transitions);
      }

      // Alarms. Tick overrun requires 2 consecutive 1 Hz windows over controlDT: a single
      // 1 s window p99 blip is the 4th-worst tick of ~333 and on a non-RT kernel reflects
      // scheduler noise; a sustained overrun is the control-health signal.
      boolean tickOverrunNow = samples > 0 && tickP99UsValue * 1000L > controlDTNs;
      alarmTickOverrun.set(tickOverrunNow && tickOverrunPrev);
      tickOverrunPrev = tickOverrunNow;
      alarmGc.set(lastPause > GC_PAUSE_ALARM_MS);
      alarmJitter.set(rustOk && jitP99 > JITTER_ALARM_US);
      boolean staleAlarm = rustOk && prevStalenessEvents >= 0 && staleEvents > prevStalenessEvents;
      alarmStaleness.set(staleAlarm);
      if (rustOk)
         prevStalenessEvents = staleEvents;

      writeJavaRegion(tickP50UsValue, tickP99UsValue, lastPause);

      summaryBuilder.setLength(0);
      summaryBuilder.append("rt[tickP50us=").append(tickP50UsValue)
                    .append(" tickP99us=").append(tickP99UsValue)
                    .append(" missN=").append(deadlineMisses.get())
                    .append(" gcLastMs=").append(String.format("%.1f", lastPause))
                    .append(" gcCumMs=").append(String.format("%.1f", gcCumMs))
                    .append(" freezeN=").append(freezes)
                    .append(" jitP50us=").append(jitP50)
                    .append(" jitP99us=").append(jitP99)
                    .append(" ovrN=").append(overruns)
                    .append(" staleN=").append(staleEvents)
                    .append(" alarmTick=").append(alarmTickOverrun.getValue())
                    .append(" alarmGc=").append(alarmGc.getValue())
                    .append(" alarmJit=").append(alarmJitter.getValue())
                    .append(" alarmStale=").append(alarmStaleness.getValue())
                    .append(']');
      return summaryBuilder.toString();
   }

   private void writeJavaRegion(long tickP50UsValue, long tickP99UsValue, double lastPauseMs)
   {
      if (buffer == null)
         return;

      javaHeartbeat++;
      javaSeq += 2;
      int odd = (int) (javaSeq - 1);
      int even = (int) javaSeq;

      INT_VIEW.setVolatile(buffer, JAVA_OFFSET + J_SEQ, odd);
      VarHandle.storeStoreFence();
      buffer.putLong(JAVA_OFFSET + J_HEARTBEAT, javaHeartbeat);
      buffer.putLong(JAVA_OFFSET + J_T_NS, System.nanoTime());
      buffer.putInt(JAVA_OFFSET + J_TICK_P50_US, (int) tickP50UsValue);
      buffer.putInt(JAVA_OFFSET + J_TICK_P99_US, (int) tickP99UsValue);
      buffer.putLong(JAVA_OFFSET + J_DEADLINE_MISS_COUNT, deadlineMisses.get());
      buffer.putDouble(JAVA_OFFSET + J_GC_LAST_PAUSE_MS, lastPauseMs);
      buffer.putDouble(JAVA_OFFSET + J_GC_TOTAL_PAUSE_MS, gcCumMs);
      buffer.putLong(JAVA_OFFSET + J_FREEZE_COUNT, freezes);
      VarHandle.releaseFence();
      INT_VIEW.setVolatile(buffer, JAVA_OFFSET + J_SEQ, even);
   }

   private static List<GarbageCollectorMXBean> selectPauseBeans(List<GarbageCollectorMXBean> all)
   {
      List<GarbageCollectorMXBean> pauses = all.stream().filter(b -> b.getName().contains("Pauses")).toList();
      return pauses.isEmpty() ? all : pauses;
   }

   private static long percentileUs(int[] histogram, long samples, double p)
   {
      if (samples == 0)
         return 0;
      long target = Math.max(1, (long) Math.ceil(p * samples));
      long cum = 0;
      for (int i = 0; i < histogram.length; i++)
      {
         cum += histogram[i];
         if (cum >= target)
            return (long) (i + 1) * TICK_BUCKET_US;
      }
      return (long) histogram.length * TICK_BUCKET_US;
   }
}
