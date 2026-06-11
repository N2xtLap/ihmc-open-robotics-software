package us.ihmc.alice5.parameters.model;

import java.io.InputStream;

public interface HumanoidURDFParameterInterface
{

   String getURDFModelName();

   String[] getResourceDirectories();

   String[] getLoggedResources();

   InputStream getURDFAsInputStream();
}
