package edu.umontreal.hatchery

import java.io.File
import java.io.FileNotFoundException

object RosInstall {
  public val defaultRosDistro = "kinetic"
  val optRos = File("/opt/ros")
  const val defaultShell = "bash"
  val setupScript = "setup.$defaultShell"

  fun getRosRoot(rosDistro: String = defaultRosDistro): File {
    var rosSetupFile = optRos.resolve("${rosDistro}/$setupScript")
    val userConfiguredRosInstall = System.getenv()["ROS_ROOT"]?.let { File(it).parentFile?.parentFile }

    return if (userConfiguredRosInstall != null) userConfiguredRosInstall
    else if (rosSetupFile.exists()) rosSetupFile
    else if (optRos.isDirectory)
      optRos.walkTopDown().maxDepth(3).first { it.name == setupScript }.parentFile
    else throw FileNotFoundException()
  }

  fun getRosSetupScript(rosDistro: String = defaultRosDistro) =
    File("${getRosRoot(rosDistro).path}/$setupScript")
}