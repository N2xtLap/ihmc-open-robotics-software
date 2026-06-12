package us.ihmc.alice5.teleop;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableClientInterface;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.util.DebugRegistry;
import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.variable.YoVariable;

import java.io.PrintWriter;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * SIM-EXT V1 debug utility: connect to a YoVariableServer and dump every variable full name to a
 * file (one per line). Usage: Alice5YoVariableDump --server <host> [--port 8008] [--out vars.txt]
 */
public class Alice5YoVariableDump implements YoVariablesUpdatedListener
{
   private final CountDownLatch done = new CountDownLatch(1);
   private final String outPath;

   public Alice5YoVariableDump(String outPath)
   {
      this.outPath = outPath;
   }

   public static void main(String[] args) throws Exception
   {
      String host = "localhost";
      int port = 8008;
      String out = "yovariables.txt";
      for (int i = 0; i < args.length - 1; i++)
      {
         if (args[i].equals("--server"))
            host = args[i + 1];
         else if (args[i].equals("--port"))
            port = Integer.parseInt(args[i + 1]);
         else if (args[i].equals("--out"))
            out = args[i + 1];
      }
      Alice5YoVariableDump dump = new Alice5YoVariableDump(out);
      YoVariableClient client = new YoVariableClient(dump);
      client.start(host, port);
      if (!dump.done.await(30, TimeUnit.SECONDS))
         System.out.println("[dump] timed out");
      client.stop();
      System.exit(0);
   }

   @Override
   public boolean updateYoVariables()
   {
      return false;
   }

   @Override
   public boolean changesVariables()
   {
      return false;
   }

   @Override
   public void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake,
                     YoVariableHandshakeParser handshakeParser, DebugRegistry debugRegistry)
   {
      try (PrintWriter writer = new PrintWriter(outPath))
      {
         for (YoVariable variable : handshakeParser.getRootRegistry().collectSubtreeVariables())
            writer.println(variable.getFullNameString());
         System.out.println("[dump] wrote " + handshakeParser.getRootRegistry().collectSubtreeVariables().size() + " variables to " + outPath);
      }
      catch (Exception e)
      {
         System.out.println("[dump] failed: " + e);
      }
      done.countDown();
   }

   @Override
   public void disconnected()
   {
   }

   @Override
   public void receivedTimestampAndData(long timestamp)
   {
   }

   @Override
   public void connected()
   {
   }

   @Override
   public void receivedCommand(DataServerCommand command, int argument)
   {
   }

   @Override
   public void receivedTimestampOnly(long timestamp)
   {
   }

   @Override
   public void setShowOverheadView(boolean showOverheadView)
   {
   }
}
