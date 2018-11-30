package edu.umontreal.hatchery.ros

import java.io.File
import java.io.FileNotFoundException

const val defaultDistro = "kinetic"
const val defaultPath = "/opt/ros"
const val rosRootEnvVar = "ROS_ROOT"
// http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT
val rootDir = System.getenv()[rosRootEnvVar]
  ?.let { File(it).parentFile?.parentFile }
  ?.absolutePath

enum class Shell(val extension: String) {
  BASH("bash"), SH("sh"), ZSH("zsh");

  override fun toString() = extension
}

val defaultShell = Shell.BASH
val shell: Shell = defaultShell

val setupScript: String
  get() = "setup.$shell"
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

// If we are inside a catkin_ws, then return the root workspace directory
fun File.getContainingRosWorkspaceIfItExists(): File =
  if (!exists()) throw FileNotFoundException("File does not exist!")
  else if (toPath().nameCount == 0) throw FileNotFoundException("No workspace found!")
  else if (parent == "src") parentFile.parentFile
  else if (isDirectory && name.endsWith("_ws")) this
  else if (parent == "_ws") parentFile
  else parentFile.getContainingRosWorkspaceIfItExists()

val rosDevelScriptPathRel = "devel/setup.$shell"
