package edu.umontreal.hatchery

import edu.umontreal.hatchery.RosInstall.Shell.BASH
import java.io.File

object RosInstall {
  const val defaultDistro = "kinetic"
  const val defaultPath = "/opt/ros"
  const val rosRootEnvVar = "ROS_ROOT"
  // http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT
  val rootDir = System.getenv()[rosRootEnvVar]
      ?.let { File(it).parentFile?.parentFile }
      ?.absolutePath

  val defaultShell = BASH
  val shell: Shell = defaultShell

  enum class Shell(val extension: String) {
    BASH("bash"), SH("sh");

    override fun toString() = extension
  }

  val setupScript: String
    get() = "setup.$defaultShell"
  var rosDistro = defaultDistro
  val defaultRosSetupScript: String
    get() = "$defaultPath/$defaultDistro/$setupScript"

  val installDir: String
    get() = if (rootDir != null) rootDir
    else if (File(defaultRosSetupScript).exists()) defaultRosSetupScript
    else if (File(defaultPath).isDirectory)
      File(defaultPath).listFiles().first().absolutePath
    else ""

  val rosSetupScript = "$installDir/$setupScript"
}