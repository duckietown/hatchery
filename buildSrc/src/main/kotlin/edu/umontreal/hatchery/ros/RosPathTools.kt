@file:Suppress("unused")

package edu.umontreal.hatchery.ros

import edu.umontreal.hatchery.ros.Shell.BASH
import java.io.File
import java.io.FileNotFoundException

const val defaultDistro = "kinetic"
const val defaultPath = "/opt/ros"
const val rosRootEnvVar = "ROS_ROOT"
// http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT
val rootDir = System.getenv()[rosRootEnvVar]
  ?.let { File(it).parentFile?.parentFile }
  ?.absolutePath

enum class Shell(private val extension: String) {
  BASH("bash"), SH("sh"), ZSH("zsh");

  override fun toString() = extension
}

val defaultShell = BASH
val shell: Shell = defaultShell

val setupScript: String
  get() = "setup.$shell"
var rosDistro = defaultDistro
val defaultRosSetupScript: String
  get() = "$defaultPath/$defaultDistro/$setupScript"

val installDir: String
  get() = rootDir ?: when {
    File(defaultRosSetupScript).exists() -> defaultRosSetupScript
    File(defaultPath).isDirectory -> File(defaultPath).listFiles().first().absolutePath
    else -> ""
  }

val rosSetupScript = "$installDir/$setupScript"

// If we are inside a catkin_ws, then return the root workspace directory
@Throws(FileNotFoundException::class)
fun File.getContainingRosWorkspaceIfItExists(query: File? = null): File = when {
  !exists() -> throw FileNotFoundException("Could not find parent ROS workspace for file: $query")
  toPath().nameCount == 0 -> throw FileNotFoundException("No workspace found!")
  parent == "src" -> parentFile.parentFile
  isDirectory && name.endsWith("_ws") -> this
  parent == "_ws" -> parentFile
  else -> parentFile.getContainingRosWorkspaceIfItExists(query ?: this)
}

val rosDevelScriptPathRel = "devel/setup.$shell"
