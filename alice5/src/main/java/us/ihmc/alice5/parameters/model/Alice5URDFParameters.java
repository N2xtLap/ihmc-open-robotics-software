package us.ihmc.alice5.parameters.model;

import jakarta.xml.bind.JAXBException;
import us.ihmc.alice5.Alice5VersionInterface;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

/**
 * ALICE5 URDF locations. The URDF and meshes are HERoEHS assets and are NOT part of this repository:
 * they live in the local assets directory which must be put on the classpath
 * (e.g. {@code -cp <ws>/assets:...} or the gradle runMain {@code -PextraClasspath} option).
 */
public class Alice5URDFParameters implements HumanoidURDFParameterInterface
{
   public static final String URDF_MODEL_NAME = "alice5";
   private static final String[] RESOURCE_DIRECTORIES = new String[] {"alice5_description/",
                                                                      "alice5_description/urdf/",
                                                                      "alice5_description/meshes/"};
   private static final String[] LOGGED_RESOURCES = {"alice5_description/"};

   public static final String URDF_FULL_BODY = "alice5_description/urdf/alice5.urdf";

   private final Collection<String> urdfModelPath;

   public Alice5URDFParameters(Alice5VersionInterface version)
   {
      urdfModelPath = version.getModelPath();
   }

   @Override
   public String getURDFModelName()
   {
      return URDF_MODEL_NAME;
   }

   @Override
   public String[] getResourceDirectories()
   {
      return RESOURCE_DIRECTORIES;
   }

   @Override
   public String[] getLoggedResources()
   {
      return LOGGED_RESOURCES;
   }

   @Override
   public InputStream getURDFAsInputStream()
   {
      List<InputStream> inputStreamList = new ArrayList<>();

      for (String path : urdfModelPath)
      {
         InputStream is = getClass().getClassLoader().getResourceAsStream(path);
         if (is == null)
            throw new RuntimeException("Unable to open robot model file: " + path
                  + " — is the local assets directory on the classpath?");
         inputStreamList.add(is);
      }

      try
      {
         URDFModel model = URDFTools.loadURDFModel(inputStreamList, Arrays.asList(RESOURCE_DIRECTORIES), getClass().getClassLoader());
         ByteArrayOutputStream bos = new ByteArrayOutputStream();
         URDFTools.saveURDFModel(bos, model);
         return new ByteArrayInputStream(bos.toByteArray());
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }
}
