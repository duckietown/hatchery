@file:Suppress("unused")

package edu.umontreal.hatchery.ros

import java.io.File
import java.io.FileNotFoundException
import java.util.concurrent.Callable
import java.util.concurrent.TimeUnit

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

const val ROS_ROOT = "ROS_ROOT"
const val ROS_DISTRO = "ROS_DISTRO"
const val PYTHONPATH = "PYTHONPATH"
const val ROS_MASTER_URI = "ROS_MASTER_URI"
const val ROS_PACKAGE_PATH = "ROS_PACKAGE_PATH"

const val defaultDistro = "kinetic"
const val defaultPath = "/opt/ros"
val defaultShell = Shell.BASH

enum class Shell(private val extension: String) {
  BASH("bash"), SH("sh"), ZSH("zsh");

  override fun toString() = extension
}

class Ros(private val distro: String = defaultDistro) {
  // http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT
  private val rootDir = System.getenv()[ROS_ROOT]
    ?.let { File(it).parentFile?.parentFile }
    ?.absolutePath


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

  interface Listable {
    val command: String
      get() = "$this list"

    val list: Callable<String>
      get() = Callable { command.runCommand() }
  }

  val pack = object: Listable {
    override fun toString() = this@Ros.toString() + if (isRos2(distro)) "pkg" else "pack"
  }

  val service = object: Listable {
    override fun toString() = this@Ros.toString() + if (isRos2(distro)) "service" else "srv"
  }

  val node = object: Listable {
    override fun toString() = "${this@Ros}node"
  }

  val topic = object: Listable {
    override fun toString() = "${this@Ros}topic"
  }

  val msg = object: Listable {
    override fun toString() = "${this@Ros}msg"
  }

  fun launch(pkg: String, launchFile: String) = object: Runnable {
    override fun run() {
      this.toString().runCommand()
    }

    private val rosWorkspace: File
      get() = try {
        File(pkg).getContainingRosWorkspaceIfItExists()
      } catch (notFound: FileNotFoundException) {
        File("")
      }

    val rosDevelScriptPathRel = "${rosWorkspace.absolutePath}/devel/setup.$shell"

    override fun toString() = """echo Sourcing $rosSetupScript && source $rosSetupScript &&
      echo 'ROS workspace directory: ${rosWorkspace.absolutePath}' &&
      cd ${rosWorkspace.absolutePath} && catkin_make &&
      echo 'Sourcing ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel' &&
      source ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel &&
      echo 'Available nodes:
            ${node.list.call()}
            Available topics:
            ${topic.list.call()}
            Available services:
            ${service.list.call()}
            Available parameters:
            ${param.list.call()}' &&
      ${this@Ros}launch $pkg $launchFile""".trimMargin()
  }

  val param = object: Listable {
    override fun toString() = "${this@Ros}param"
  }

  override fun toString() = if (isRos2(distro)) "ros2 " else "ros"
}

fun isRos2(distro: String) =
  "ardent" in distro || "bouncy" in distro || "crystal" in distro

fun String.runCommand(): String {
  val parts = split("\\s".toRegex())
  val proc = ProcessBuilder(*parts.toTypedArray())
    .redirectOutput(ProcessBuilder.Redirect.PIPE)
    .redirectError(ProcessBuilder.Redirect.PIPE)
    .start()

  proc.waitFor(60, TimeUnit.SECONDS)
  return proc.inputStream.bufferedReader().readText()
}