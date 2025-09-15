import us.ihmc.jros2.generator.task.jros2GenTask

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.jros2.generator") version "1.0.3"
}

ihmc {
   loadProductProperties("../product.properties")
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:jros2:1.0.1")
}

tasks.register<jros2GenTask>("generateMessages") {
   packagePaths = listOf(
      projectDir.resolve("ros2").resolve("atlas_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("controller_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("ihmc_common_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("toolbox_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("behavior_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("perception_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("exoskeleton_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("system_monitor_msgs").absolutePath,
      projectDir.resolve("ros2").resolve("test_msgs").absolutePath,
   )

   outputDir = sourceSets["main"].java.srcDirs.find { it.name == "java" }.toString()
}