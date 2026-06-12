package us.ihmc.alice5.bridge;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

/**
 * Java side of the ALICE5 WBC shared-memory bridge (docs/bridge_spec.md rev.2).
 * Opens /dev/shm/alice5_wbc (open-only, the robot-side process is the creator),
 * validates the header (magic, ABI, layout hash, offsets) and provides seqlock
 * based state reads and command writes. Little-endian fixed.
 */
public class Alice5ShmBridge
{
   public static final String SHM_PATH = "/dev/shm/alice5_wbc";
   public static final String JAVA_LOCK_PATH = "/dev/shm/alice5_wbc.java.lock";

   public static final int SHM_SIZE = 4096;
   public static final int N_JOINTS = 23;
   public static final long MAGIC = 0x414C494345355742L; // "ALICE5WB" little-endian
   public static final int ABI_VERSION = 2; // v2 (M3.5+M3.7 single bump): validity bit0/1=wrench, 2/3=CoP, 4=IMU
   public static final int STATE_OFFSET = 64;
   public static final int STATE_SIZE = 1536;
   public static final int CMD_OFFSET = 1600;
   public static final int CMD_SIZE = 1536;

   public static final int MODE_IDLE = 0;
   public static final int MODE_POSITION = 1;
   public static final int MODE_DAMPING_REQUEST = 2;

   public static final int SAFETY_INIT = 0;
   public static final int SAFETY_NORMAL = 1;
   public static final int SAFETY_DAMPING = 2;
   public static final int SAFETY_DISABLED = 3;

   // Header field offsets (bytes from start of mapping).
   private static final int HDR_MAGIC = 0;
   private static final int HDR_ABI = 8;
   private static final int HDR_N_JOINTS = 12;
   private static final int HDR_LAYOUT_HASH = 16;
   private static final int HDR_STATE_OFFSET = 24;
   private static final int HDR_STATE_SIZE = 28;
   private static final int HDR_CMD_OFFSET = 32;
   private static final int HDR_CMD_SIZE = 36;

   // State block field offsets (bytes from STATE_OFFSET).
   private static final int S_SEQ = 0;
   private static final int S_SAFETY = 4;
   private static final int S_T_MONO_NS = 8;
   private static final int S_CYCLE = 16;
   private static final int S_Q = 24;
   private static final int S_QD = 208;
   private static final int S_TAU = 392;
   private static final int S_STATUSWORD = 576;
   private static final int S_FAULT_BITS = 624;
   private static final int S_IMU_QUAT = 632;
   private static final int S_IMU_GYRO = 664;
   private static final int S_IMU_ACC = 688;
   private static final int S_FOOT_FT = 712;
   private static final int S_FOOT_FT_VALIDITY = 808;

   // Cmd block field offsets (bytes from CMD_OFFSET).
   private static final int C_SEQ = 0;
   private static final int C_MODE = 4;
   private static final int C_HEARTBEAT = 8;
   private static final int C_T_JAVA_NS = 16;
   private static final int C_ENABLE_MASK = 24;
   private static final int C_Q_DES = 32;
   private static final int C_QD_DES = 216;
   private static final int C_TAU_FF = 400;
   private static final int C_KP = 584;
   private static final int C_KD = 768;

   /** Must match the machine-generated string hashed by the Rust creator (FNV-1a 64). */
   private static final String LAYOUT_STRING = "v2:n=23;s.seq:0;s.safety:4;s.t:8;s.cycle:16;s.q:24;s.qd:208;s.tau:392;s.sw:576;"
         + "s.fault:624;s.quat:632;s.gyro:664;s.acc:688;s.ft:712;s.ftv:808;"
         + "c.seq:0;c.mode:4;c.hb:8;c.tj:16;c.en:24;c.qdes:32;c.qddes:216;c.tauff:400;c.kp:584;c.kd:768;";

   private static final long OPEN_DEADLINE_NS = 5_000_000_000L;
   private static final long OPEN_POLL_MS = 100L;
   private static final int SEQLOCK_MAX_RETRIES = 8;

   private static final VarHandle INT_VIEW = MethodHandles.byteBufferViewVarHandle(int[].class, ByteOrder.LITTLE_ENDIAN);

   private FileChannel shmChannel;
   private MappedByteBuffer buffer;
   private FileChannel lockChannel;
   private FileLock javaLock;
   private long commandCounter = 0;

   /** Plain-Java snapshot of the state block. */
   public static class StateSnapshot
   {
      public long seq;
      public int safetyState;
      public long tMonoNs;
      public long cycle;
      public final double[] q = new double[N_JOINTS];
      public final double[] qd = new double[N_JOINTS];
      public final double[] tau = new double[N_JOINTS];
      public final int[] statusword = new int[N_JOINTS];
      public long faultBits;
      public final double[] imuQuat = new double[4]; // w, x, y, z
      public final double[] imuGyro = new double[3];
      public final double[] imuAcc = new double[3];
      public final double[][] footFt = new double[2][6]; // [left, right], each F xyz then T xyz
      public int footFtValidity;

      public void set(StateSnapshot other)
      {
         seq = other.seq;
         safetyState = other.safetyState;
         tMonoNs = other.tMonoNs;
         cycle = other.cycle;
         System.arraycopy(other.q, 0, q, 0, N_JOINTS);
         System.arraycopy(other.qd, 0, qd, 0, N_JOINTS);
         System.arraycopy(other.tau, 0, tau, 0, N_JOINTS);
         System.arraycopy(other.statusword, 0, statusword, 0, N_JOINTS);
         faultBits = other.faultBits;
         System.arraycopy(other.imuQuat, 0, imuQuat, 0, 4);
         System.arraycopy(other.imuGyro, 0, imuGyro, 0, 3);
         System.arraycopy(other.imuAcc, 0, imuAcc, 0, 3);
         System.arraycopy(other.footFt[0], 0, footFt[0], 0, 6);
         System.arraycopy(other.footFt[1], 0, footFt[1], 0, 6);
         footFtValidity = other.footFtValidity;
      }
   }

   public boolean isOpen()
   {
      return buffer != null;
   }

   /**
    * Acquires the single-instance Java lock, then opens and validates the shared memory.
    * Polls every 100 ms while the segment is missing or not yet published; throws a
    * RuntimeException after a 5 s deadline (caller is expected to exit with code 4).
    */
   public synchronized void open()
   {
      if (isOpen())
         return;

      acquireJavaLock();

      long deadline = System.nanoTime() + OPEN_DEADLINE_NS;
      String lastError = "shared memory not checked yet";

      while (true)
      {
         lastError = tryOpenOnce();
         if (lastError == null)
            return;

         if (System.nanoTime() > deadline)
         {
            releaseJavaLock();
            throw new RuntimeException("Failed to open " + SHM_PATH + " within 5 s: " + lastError);
         }

         try
         {
            Thread.sleep(OPEN_POLL_MS);
         }
         catch (InterruptedException e)
         {
            Thread.currentThread().interrupt();
            releaseJavaLock();
            throw new RuntimeException("Interrupted while waiting for " + SHM_PATH, e);
         }
      }
   }

   /** @return null on success, otherwise a description of why the segment is not usable yet. */
   private String tryOpenOnce()
   {
      Path path = Path.of(SHM_PATH);
      if (!Files.exists(path))
         return "file does not exist";

      try
      {
         FileChannel channel = FileChannel.open(path, StandardOpenOption.READ, StandardOpenOption.WRITE);
         if (channel.size() != SHM_SIZE)
         {
            long size = channel.size();
            channel.close();
            return "unexpected size " + size + " (expected " + SHM_SIZE + ")";
         }

         MappedByteBuffer mapped = channel.map(FileChannel.MapMode.READ_WRITE, 0, SHM_SIZE);
         mapped.order(ByteOrder.LITTLE_ENDIAN);

         String headerError = validateHeader(mapped);
         if (headerError != null)
         {
            channel.close();
            return headerError;
         }

         shmChannel = channel;
         buffer = mapped;
         return null;
      }
      catch (IOException e)
      {
         return "I/O error: " + e.getMessage();
      }
   }

   private String validateHeader(MappedByteBuffer mapped)
   {
      long magic = mapped.getLong(HDR_MAGIC);
      if (magic != MAGIC)
         return String.format("magic mismatch: found 0x%016X, expected 0x%016X (creator may not have published yet)", magic, MAGIC);

      int abi = mapped.getInt(HDR_ABI);
      if (abi != ABI_VERSION)
         return "ABI version mismatch: found " + abi + ", expected " + ABI_VERSION;

      int nJoints = mapped.getInt(HDR_N_JOINTS);
      if (nJoints != N_JOINTS)
         return "n_joints mismatch: found " + nJoints + ", expected " + N_JOINTS;

      long expectedHash = fnv1a64(LAYOUT_STRING.getBytes(StandardCharsets.US_ASCII));
      long layoutHash = mapped.getLong(HDR_LAYOUT_HASH);
      if (layoutHash != expectedHash)
         return String.format("layout_hash mismatch: found 0x%016X, expected 0x%016X", layoutHash, expectedHash);

      int stateOffset = mapped.getInt(HDR_STATE_OFFSET);
      if (stateOffset != STATE_OFFSET)
         return "state_offset mismatch: found " + stateOffset + ", expected " + STATE_OFFSET;

      int stateSize = mapped.getInt(HDR_STATE_SIZE);
      if (stateSize != STATE_SIZE)
         return "state_size mismatch: found " + stateSize + ", expected " + STATE_SIZE;

      int cmdOffset = mapped.getInt(HDR_CMD_OFFSET);
      if (cmdOffset != CMD_OFFSET)
         return "cmd_offset mismatch: found " + cmdOffset + ", expected " + CMD_OFFSET;

      int cmdSize = mapped.getInt(HDR_CMD_SIZE);
      if (cmdSize != CMD_SIZE)
         return "cmd_size mismatch: found " + cmdSize + ", expected " + CMD_SIZE;

      return null;
   }

   private void acquireJavaLock()
   {
      if (javaLock != null)
         return;

      try
      {
         lockChannel = FileChannel.open(Path.of(JAVA_LOCK_PATH), StandardOpenOption.CREATE, StandardOpenOption.WRITE);
         javaLock = lockChannel.tryLock();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Failed to open Java instance lock " + JAVA_LOCK_PATH, e);
      }

      if (javaLock == null)
         throw new RuntimeException("Another Java bridge instance already holds " + JAVA_LOCK_PATH + ", refusing to start.");
   }

   private void releaseJavaLock()
   {
      try
      {
         if (javaLock != null)
            javaLock.release();
         if (lockChannel != null)
            lockChannel.close();
      }
      catch (IOException e)
      {
         // Best effort on shutdown.
      }
      javaLock = null;
      lockChannel = null;
   }

   /**
    * Seqlock read of the state block into the given snapshot.
    *
    * @return true if a consistent snapshot was read, false if the retry budget was exhausted
    *         (in which case the snapshot content is undefined and must not be used).
    */
   public boolean readState(StateSnapshot snapshot)
   {
      if (!isOpen())
         return false;

      MappedByteBuffer buffer = this.buffer;

      for (int attempt = 0; attempt < SEQLOCK_MAX_RETRIES; attempt++)
      {
         int s1 = (int) INT_VIEW.getVolatile(buffer, STATE_OFFSET + S_SEQ);
         if ((s1 & 1) != 0)
            continue;

         snapshot.safetyState = buffer.getInt(STATE_OFFSET + S_SAFETY);
         snapshot.tMonoNs = buffer.getLong(STATE_OFFSET + S_T_MONO_NS);
         snapshot.cycle = buffer.getLong(STATE_OFFSET + S_CYCLE);
         for (int i = 0; i < N_JOINTS; i++)
         {
            snapshot.q[i] = buffer.getDouble(STATE_OFFSET + S_Q + 8 * i);
            snapshot.qd[i] = buffer.getDouble(STATE_OFFSET + S_QD + 8 * i);
            snapshot.tau[i] = buffer.getDouble(STATE_OFFSET + S_TAU + 8 * i);
            snapshot.statusword[i] = buffer.getShort(STATE_OFFSET + S_STATUSWORD + 2 * i) & 0xFFFF;
         }
         snapshot.faultBits = buffer.getLong(STATE_OFFSET + S_FAULT_BITS);
         for (int i = 0; i < 4; i++)
            snapshot.imuQuat[i] = buffer.getDouble(STATE_OFFSET + S_IMU_QUAT + 8 * i);
         for (int i = 0; i < 3; i++)
         {
            snapshot.imuGyro[i] = buffer.getDouble(STATE_OFFSET + S_IMU_GYRO + 8 * i);
            snapshot.imuAcc[i] = buffer.getDouble(STATE_OFFSET + S_IMU_ACC + 8 * i);
         }
         for (int side = 0; side < 2; side++)
         {
            for (int i = 0; i < 6; i++)
               snapshot.footFt[side][i] = buffer.getDouble(STATE_OFFSET + S_FOOT_FT + 8 * (6 * side + i));
         }
         snapshot.footFtValidity = buffer.getInt(STATE_OFFSET + S_FOOT_FT_VALIDITY);

         VarHandle.acquireFence();
         int s2 = (int) INT_VIEW.getVolatile(buffer, STATE_OFFSET + S_SEQ);

         if (s1 == s2)
         {
            snapshot.seq = s1 & 0xFFFFFFFFL;
            return true;
         }
      }

      return false;
   }

   /**
    * Seqlock write of the command block. Single writer (enforced by the Java lock file).
    * All arrays must have length {@value #N_JOINTS}.
    */
   public void writeCommand(int mode, long heartbeat, int enableMask, double[] qDes, double[] qdDes, double[] tauFf, double[] kp, double[] kd)
   {
      if (!isOpen())
         throw new IllegalStateException("Shared memory is not open.");

      MappedByteBuffer buffer = this.buffer;

      commandCounter++;
      int odd = (int) (2 * commandCounter - 1);
      int even = (int) (2 * commandCounter);

      INT_VIEW.setVolatile(buffer, CMD_OFFSET + C_SEQ, odd);
      VarHandle.storeStoreFence();

      buffer.putInt(CMD_OFFSET + C_MODE, mode);
      buffer.putLong(CMD_OFFSET + C_HEARTBEAT, heartbeat);
      buffer.putLong(CMD_OFFSET + C_T_JAVA_NS, System.nanoTime());
      buffer.putInt(CMD_OFFSET + C_ENABLE_MASK, enableMask);
      for (int i = 0; i < N_JOINTS; i++)
      {
         buffer.putDouble(CMD_OFFSET + C_Q_DES + 8 * i, qDes[i]);
         buffer.putDouble(CMD_OFFSET + C_QD_DES + 8 * i, qdDes[i]);
         buffer.putDouble(CMD_OFFSET + C_TAU_FF + 8 * i, tauFf[i]);
         buffer.putDouble(CMD_OFFSET + C_KP + 8 * i, kp[i]);
         buffer.putDouble(CMD_OFFSET + C_KD + 8 * i, kd[i]);
      }

      VarHandle.releaseFence();
      INT_VIEW.setVolatile(buffer, CMD_OFFSET + C_SEQ, even);
   }

   public synchronized void close()
   {
      try
      {
         if (shmChannel != null)
            shmChannel.close();
      }
      catch (IOException e)
      {
         // Best effort on shutdown.
      }
      shmChannel = null;
      buffer = null;
      releaseJavaLock();
   }

   public static long fnv1a64(byte[] data)
   {
      long hash = 0xcbf29ce484222325L;
      for (byte b : data)
      {
         hash ^= (b & 0xFFL);
         hash *= 0x100000001b3L;
      }
      return hash;
   }

   public static String safetyStateName(int safetyState)
   {
      switch (safetyState)
      {
         case SAFETY_INIT:
            return "INIT";
         case SAFETY_NORMAL:
            return "NORMAL";
         case SAFETY_DAMPING:
            return "DAMPING";
         case SAFETY_DISABLED:
            return "DISABLED";
         default:
            return "UNKNOWN(" + safetyState + ")";
      }
   }
}
