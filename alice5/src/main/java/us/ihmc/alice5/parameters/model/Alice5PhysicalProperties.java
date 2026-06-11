package us.ihmc.alice5.parameters.model;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * ALICE5 physical properties.
 *
 * All numeric values are HERoEHS-derived (CAD/MJCF) and therefore must NOT be hardcoded in this
 * public repository. They are loaded at runtime from {@code alice5_description/alice5_physical.properties}
 * which lives in the local (git-ignored) assets directory, injected on the classpath.
 */
public class Alice5PhysicalProperties
{
   public static final String PROPERTIES_RESOURCE = "alice5_description/alice5_physical.properties";

   private final double ankleHeight, footLength, footBack, footForward, footWidth, thighLength, shinLength;
   private final double footLengthForControl, footWidthForControl, footForwardForControl, footBackForControl;
   private final double contactFrontX, contactBackX, contactY;
   private final double imuPosX, imuPosY, imuPosZ;
   private final double jointVelocityLimitDefault;

   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();

   public Alice5PhysicalProperties()
   {
      Properties props = new Properties();
      try (InputStream is = getClass().getClassLoader().getResourceAsStream(PROPERTIES_RESOURCE))
      {
         if (is == null)
            throw new RuntimeException("Cannot find " + PROPERTIES_RESOURCE
                  + " on the classpath. Add the local assets directory to the classpath (it is intentionally not in this repo).");
         props.load(is);
      }
      catch (IOException e)
      {
         throw new RuntimeException("Failed to load " + PROPERTIES_RESOURCE, e);
      }

      ankleHeight = get(props, "ankleHeight");
      footLength = get(props, "actualFootLength");
      footWidth = get(props, "actualFootWidth");
      footLengthForControl = get(props, "footLengthForControl");
      footWidthForControl = get(props, "footWidthForControl");
      footBackForControl = get(props, "footBackForControl");
      footForwardForControl = get(props, "footForwardForControl");
      thighLength = get(props, "thighLength");
      shinLength = get(props, "shinLength");
      contactFrontX = get(props, "contactFrontX");
      contactBackX = get(props, "contactBackX");
      contactY = get(props, "contactY");
      imuPosX = get(props, "imuPosX");
      imuPosY = get(props, "imuPosY");
      imuPosZ = get(props, "imuPosZ");
      jointVelocityLimitDefault = get(props, "jointVelocityLimitDefault");

      double soleToAnkleX = get(props, "soleToAnkleX");
      double soleToAnkleY = get(props, "soleToAnkleY");
      double soleToAnkleZ = get(props, "soleToAnkleZ");
      footBack = -(soleToAnkleX + contactBackX) + 0.0; // ankle to control polygon rear edge
      footForward = soleToAnkleX + contactFrontX;

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         soleToAnkleFrame.getTranslation().set(new Vector3D(soleToAnkleX, soleToAnkleY, soleToAnkleZ));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }
   }

   private static double get(Properties props, String key)
   {
      String value = props.getProperty(key);
      if (value == null)
         throw new RuntimeException("Missing key '" + key + "' in " + PROPERTIES_RESOURCE);
      return Double.parseDouble(value.trim());
   }

   public double getModelSizeScale()
   {
      return 1.0;
   }

   public double getModelMassScalePower()
   {
      return 1.0;
   }

   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public double getActualFootLength()
   {
      return footLength;
   }

   public double getActualFootWidth()
   {
      return footWidth;
   }

   public double getActualFootBack()
   {
      return footBack;
   }

   public double getFootLengthForControl()
   {
      return footLengthForControl;
   }

   public double getFootBackForControl()
   {
      return footBackForControl;
   }

   public double getFootForwardForControl()
   {
      return footForwardForControl;
   }

   public double getFootBack()
   {
      return footBack;
   }

   public double getFootForward()
   {
      return footForward;
   }

   public double getFootWidthForControl()
   {
      return footWidthForControl;
   }

   public double getToeWidthForControl()
   {
      return footWidthForControl;
   }

   public double getThighLength()
   {
      return thighLength;
   }

   public double getShinLength()
   {
      return shinLength;
   }

   public double getLegLength()
   {
      return thighLength + shinLength;
   }

   /** Foot sole contact point coordinates in sole frame (from loadcell positions). */
   public double getContactFrontX()
   {
      return contactFrontX;
   }

   public double getContactBackX()
   {
      return contactBackX;
   }

   public double getContactY()
   {
      return contactY;
   }

   public Vector3D getImuPositionInPelvis()
   {
      return new Vector3D(imuPosX, imuPosY, imuPosZ);
   }

   public double getJointVelocityLimitDefault()
   {
      return jointVelocityLimitDefault;
   }

   public SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms()
   {
      return soleToAnkleFrameTransforms;
   }

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide side)
   {
      return new RigidBodyTransform();
   }
}
