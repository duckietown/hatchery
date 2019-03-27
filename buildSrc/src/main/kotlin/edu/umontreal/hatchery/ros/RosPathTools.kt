@file:Suppress("unused")

package edu.umontreal.hatchery.ros

import edu.umontreal.hatchery.ros.BuildSystem.*
import edu.umontreal.hatchery.ros.Distro.*
import edu.umontreal.hatchery.ros.RosEnv.*
import edu.umontreal.hatchery.ros.Shell.*
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
  ROS_ROOT, ROS_DISTRO, PYTHONPATH, ROS_MASTER_URI, ROS_PACKAGE_PATH, CATKIN_SHELL
}

const val defaultDistro = "kinetic"
const val baseRosPath = "/opt/ros"
val defaultShell =
  Shell.values().firstOrNull { it.name == System.getenv("$CATKIN_SHELL") } ?: sh

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
  bash, sh, zsh;

  override fun toString() = name.toLowerCase()
}

enum class Distro {
  electric, fuerte,
  groovy, hydro, indigo, jade, kinetic, lunar, melodic,
  ardent, bouncy, crystal;
}

enum class BuildSystem(val command: String) {
  catkin("catkin_make"), rosbuild("rosmake"), colcon("colcon build");
}

class Ros(val setupScript: String = defaultRosSetupScript) {
  val rosSetupScriptFile = File(setupScript)

  val shell = try {
    Shell.valueOf(setupScript.split(".").last())
  } catch (e: Exception) {
    sh
  }

  val distro: Distro
  val pythonPath: File
  val buildSystem: BuildSystem

  init {
    val info = runCommand(shell.name, "-c",
      """. $setupScript && echo "
      |$$ROS_DISTRO
      |$$PYTHONPATH"""".trimMargin()).lines().dropLastWhile { it.isBlank() }.takeLast(2)
    distro = Distro.valueOf(info.first())
    pythonPath = File(info.last())
    buildSystem = when (distro) {
      electric, fuerte -> rosbuild
      groovy, hydro, indigo, jade, kinetic, lunar, melodic -> catkin
      ardent, bouncy, crystal-> colcon
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
    override fun toString() = this@Ros.toString() + if (isRos2(setupScript)) "pkg" else "pack"
  }

  val service = object : Listable(this) {
    override fun toString() = this@Ros.toString() + if (isRos2(setupScript)) "service" else "srv"
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

    override fun toString() = """echo Sourcing $setupScript && . $setupScript &&
      echo 'ROS workspace directory: ${rosWorkspace.absolutePath}' &&
      cd ${rosWorkspace.absolutePath} &&
      ${buildSystem.command} &&
      echo 'Sourcing ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel' &&
      . ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel &&
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

  override fun toString() = if (isRos2(setupScript)) "ros2 " else "ros"
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