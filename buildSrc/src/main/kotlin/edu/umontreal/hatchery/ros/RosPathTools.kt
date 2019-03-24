@file:Suppress("unused")

package edu.umontreal.hatchery.ros

import edu.umontreal.hatchery.ros.RosEnv.*
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

enum class RosEnv {
  ROS_ROOT, ROS_DISTRO, PYTHONPATH, ROS_MASTER_URI, ROS_PACKAGE_PATH
}

const val defaultDistro = "kinetic"
const val baseRosPath = "/opt/ros"
val defaultShell = Shell.BASH

private val defaultRootDir =
  System.getenv("$ROS_ROOT")
    ?.let { File(it).parentFile?.parentFile }
    ?.absolutePath

val defaultInstallDir
  get() = defaultRootDir ?: if (File(baseRosPath).isDirectory)
    File(baseRosPath).listFiles().first().absolutePath
  else null

val defaultRosSetupScript =
  if (defaultInstallDir != null) "$defaultInstallDir/setup.$defaultShell" else ""

enum class Shell {
  BASH, SH, ZSH;

  override fun toString() = name.toLowerCase()
}

enum class Distro {
  ELECTRIC, FUERTE, GROOVY, HYDRO, INDIGO, JADE, KINETIC, LUNAR, MELODIC, BOUNCY, CRYSTAL, UNKNOWN;
}

class Ros(val rosSetupScript: String = defaultRosSetupScript) {
  val rosSetupScriptFile = File(rosSetupScript)

  val shell = try {
    Shell.valueOf(rosSetupScript.split(".").last())
  } catch (e: Exception) {
    Shell.SH
  }

  val distro: Distro
  val pythonPath: File

  init {
    val info = runCommand(shell.name, "-c",
      """source $rosSetupScript && echo '
      |$$ROS_DISTRO
      |$$PYTHONPATH""".trimMargin()).lines().takeLast(2)
    pythonPath = File(info.first())
    distro = try {
      Distro.valueOf(info.last().toUpperCase())
    } catch (e: Exception) {
      Distro.UNKNOWN
    }
  }

  // http://wiki.ros.org/ROS/EnvironmentVariables#ROS_ROOT
  open class Listable(val ros: Ros) {
    val command: String
      get() = "$this list"

    val list: Callable<String>
      get() = object : Callable<String> {
        override fun call() = runCommand(ros.shell.name, "-c", command)
        override fun toString() = command
      }
  }

  val pack = object : Listable(this) {
    override fun toString() = this@Ros.toString() + if (isRos2(rosSetupScript)) "pkg" else "pack"
  }

  val service = object : Listable(this) {
    override fun toString() = this@Ros.toString() + if (isRos2(rosSetupScript)) "service" else "srv"
  }

  val node = object : Listable(this) {
    override fun toString() = "${this@Ros}node"
  }

  val topic = object : Listable(this) {
    override fun toString() = "${this@Ros}topic"
  }

  val msg = object : Listable(this) {
    override fun toString() = "${this@Ros}msg"
  }

  fun launch(pkg: String, launchFile: String) = object : Runnable {
    override fun run() {
      runCommand(shell.name, "-c", toString())
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
      echo 'Available nodes:' &&
      ${node.list} &&
      echo 'Available topics:' &&
      ${topic.list} &&
      echo 'Available services:' &&
      ${service.list} &&
      echo 'Available parameters:' &&
      ${param.list}' &&
      ${this@Ros}launch $pkg $launchFile""".trimMargin()
  }

  val param = object : Listable(this) {
    override fun toString() = "${this@Ros}param"
  }

  override fun toString() = if (isRos2(rosSetupScript)) "ros2 " else "ros"
}

fun runCommand(vararg commands: String): String {
  val proc = ProcessBuilder(*commands)
    .redirectOutput(ProcessBuilder.Redirect.PIPE)
    .redirectError(ProcessBuilder.Redirect.PIPE)
    .start()

  proc.waitFor(60, TimeUnit.SECONDS)
  return proc.inputStream.bufferedReader().readText()
}

fun isRos2(distro: String) =
  "ardent" in distro || "bouncy" in distro || "crystal" in distro