@file:Suppress("EnumEntryName", "unused")

package edu.umontreal.hatchery.ros

import edu.umontreal.hatchery.ros.BuildSystem.*
import edu.umontreal.hatchery.ros.RosEnv.*
import edu.umontreal.hatchery.ros.Shell.sh
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

const val baseRosPath = "/opt/ros"
val defaultShell =
  Shell.values().firstOrNull { it.name == System.getenv("$CATKIN_SHELL") } ?: sh

private val defaultRootDir =
  System.getenv("$ROS_ROOT")
    ?.let { File(it).parentFile?.parentFile }
    ?.absolutePath

val defaultInstallDir =
  defaultRootDir ?: if (File(baseRosPath).isDirectory)
    File(baseRosPath).listFiles().first().absolutePath
  else null

val defaultRosSetupScript =
  if (defaultInstallDir != null) "$defaultInstallDir/setup.$defaultShell" else ""

enum class Shell {
  bash, sh, zsh;

  override fun toString() = name.toLowerCase()
}

enum class Distro(val version: Int) {
  electric(0), fuerte(0),
  groovy(1), hydro(1), indigo(1), jade(1), kinetic(1), lunar(1), melodic(1),
  ardent(2), bouncy(2), crystal(2), dashing(2);
}

enum class BuildSystem(val command: String) {
  catkin("catkin_make"), rosbuild("rosmake"), colcon("colcon build");
}

class Ros(val setupScript: String = defaultRosSetupScript) {
  val shell: Shell
  val setupScriptFile: File
  val distro: Distro
  val pythonPath: File
  val buildSystem: BuildSystem
  val command: String
  val env: Map<String, String>
  val packages: Map<String, String> by lazy {
    pack.list.call().map {
      if (it.contains(" "))
        Pair(it.substringBefore(" "), it.substringAfter(" ") + "/package.xml")
      else
        Pair(it, "")
    }.toMap()
  }

  init {
    shell = try {
      Shell.valueOf(setupScript.split(".").last())
    } catch (e: Exception) {
      sh
    }
    setupScriptFile = File(setupScript)
    // http://wiki.ros.org/ROS/EnvironmentVariables
    env = runCommand(shell.name, "-c", ". $setupScript && env")
      .lines().mapNotNull { it.split("=").zipWithNext().firstOrNull() }.toMap()
    distro = Distro.valueOf(env["$ROS_DISTRO"]!!)
    pythonPath = File(env["$PYTHONPATH"])
    buildSystem = when (distro.version) {
      0 -> rosbuild
      1 -> catkin
      else -> colcon
    }
    command = when (distro.version) {
      0, 1 -> "ros"
      else -> "ros2 "
    }
  }

  class Command(val shell: String, val command: String) : () -> String {
    override fun invoke(): String = runCommand(shell, "-c", command)

    override fun toString() = command
  }

  open class Listable(val ros: Ros) {
    val command = Command(ros.shell.name, ". ${ros.setupScript} && $this list")

    val list: Callable<List<String>> = object : Callable<List<String>> {
      override fun call() = command().lines().dropLastWhile { it.isBlank() }

      override fun toString() = command.toString()
    }
  }

  val pack = object : Listable(this) {
    override fun toString() = "$ros" + if (1 < distro.version) "pkg" else "pack"
  }

  val service = object : Listable(this) {
    override fun toString() = "$ros" + if (1 < distro.version) "service" else "srv"
  }

  val node = object : Listable(this) {
    override fun toString() = "${ros}node"
  }

  val topic = object : Listable(this) {
    override fun toString() = "${ros}topic"
  }

  val msg = object : Listable(this) {
    override fun toString() = "${ros}msg"
  }

  fun launch(rosPackage: String, launchFile: String) = object : Runnable {
    override fun run() {
      runCommand(shell.name, "-c", toString())
    }

    private val rosWorkspace: File = try {
      File(rosPackage).getContainingRosWorkspaceIfItExists()
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
      ${param.list} &&
      ${this@Ros}launch $rosPackage $launchFile""".trimMargin()
  }

  val param = object : Listable(this) {
    override fun toString() = "${ros}param"
  }

  override fun toString() = command
}

fun runCommand(vararg commands: String): String {
  val proc = ProcessBuilder(*commands)
    .redirectOutput(ProcessBuilder.Redirect.PIPE)
    .redirectError(ProcessBuilder.Redirect.PIPE)
    .start()

  proc.waitFor(60, TimeUnit.SECONDS)
  return proc.inputStream.bufferedReader().readText()
}