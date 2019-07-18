@file:Suppress("EnumEntryName", "unused")

package edu.umontreal.hatchery.ros

import com.intellij.openapi.project.Project
import com.intellij.openapi.project.ProjectManager
import edu.umontreal.hatchery.ros.BuildSystem.*
import edu.umontreal.hatchery.ros.RosEnv.*
import edu.umontreal.hatchery.ros.Shell.sh
import edu.umontreal.hatchery.util.findFilesByRelativePath
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

// http://wiki.ros.org/ROS/EnvironmentVariables
enum class RosEnv {
  // Required ROS Environment Variables
  ROS_ROOT,
  ROS_DISTRO, PYTHONPATH, ROS_MASTER_URI,
  // Additional PATH Environment Variables
  ROS_PACKAGE_PATH,
  CATKIN_SHELL,
  // System Data Environment Variables
  ROS_HOME,
  ROS_LOG_DIR, ROS_TEST_RESULTS_DIR, ROS_CACHE_TIMEOUT,
  // Additional Bash Environment Variables
  ROS_LOCATIONS,
  ROS_WORKSPACE,
  // Node Environment Variables
  ROS_IP,
  ROS_HOSTNAME, ROS_NAMESPACE, ROSCONSOLE_CONFIG_FILE,
  // Console Output Formatting
  ROS_PYTHON_LOG_CONFIG_FILE,
  // Build System Environment Variables
  ROS_BOOST_ROOT,
  ROS_PARALLEL_JOBS, ROS_LANG_DISABLE, ROS_OS_OVERRIDE
}

val pathSeparator = File.separator ?: "/"

val defaultRosInstallPath =
  if (System.getProperty("os.name").toLowerCase().startsWith("windows"))
    "c:${pathSeparator}opt${pathSeparator}ros"
  else "${pathSeparator}opt${pathSeparator}ros"

val defaultShell =
  Shell.values().firstOrNull { it.name == System.getenv("$CATKIN_SHELL") } ?: sh

private val ROS_ROOT_PATH =
  System.getenv("$ROS_ROOT")
    ?.let { File(it).parentFile?.parentFile }
    ?.absolutePath

val defaultRosSetupScript =
  ROS_ROOT_PATH ?: if (File(defaultRosInstallPath).isDirectory)
    File(defaultRosInstallPath).listFiles { distroDir: File? ->
      distroDir?.isDirectory ?: false
    }.map { dir ->
      dir.listFiles { setupScript ->
        setupScript.nameWithoutExtension == "setup" &&
          setupScript.extension in Shell.values().map { it.name }
      }.toList()
    }.flatten().minBy { it.extension != "sh" }?.absolutePath
  else null

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
  catkin_tools("catkin build"), catkin("catkin_make"), rosbuild("rosmake"), colcon("colcon build");
}

fun getSetupScriptInProject() =
  ProjectManager.getInstance().openProjects.map { project ->
    findFilesByRelativePath(project, "setup.sh")
      .mapNotNull { it?.virtualFile?.canonicalPath }
  }.flatten().firstOrNull()

@Suppress("LeakingThis", "MemberVisibilityCanBePrivate")
class Ros(val setupScript: String = defaultRosSetupScript
  ?: getSetupScriptInProject() ?: "") {
  val shell: Shell
  val setupScriptFile: File?
  val distro: Distro
  val pythonPath: File?
  val buildSystem: BuildSystem
  val command: String
  val env: Map<String, String>
  val masterUri: String?
  val packagePath: File?

  val packages: Map<String, String> by lazy {
    pack.list.call().map {
      if (it.contains(" "))
        Pair(it.substringBefore(" "), it.substringAfter(" ") + "${pathSeparator}package.xml")
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
    setupScriptFile = File(setupScript).let { if (it.exists()) it else null }
    // http://wiki.ros.org/ROS/EnvironmentVariables
    env = if (System.getenv()["$ROS_ROOT"] == setupScript) System.getenv()
    else runCommandAndFetchOutput(emptyMap(), shell.name, "-c", ". $setupScript && env")
      .lines().mapNotNull { it.split("=").zipWithNext().firstOrNull() }.toMap()
    distro = Distro.valueOf(env["$ROS_DISTRO"] ?: Distro.kinetic.name)
    masterUri = env["$ROS_MASTER_URI"]
    pythonPath = env["$PYTHONPATH"]?.let { File(it) }
    packagePath = env["$ROS_PACKAGE_PATH"]?.let { File(it) }
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

  fun runInBackground(command: String) = runCommandInBackground(env, shell.name, "-c", ". $setupScript && $command")

  fun getOutput(command: String) = runCommandAndFetchOutput(env, shell.name, "-c", ". $setupScript && $command")

  abstract class Listable(val ros: Ros) {
    val list: Callable<List<String>> = object : Callable<List<String>> {
      val command = "${this@Listable} list"

      override fun call() = ros.getOutput(command).lines().dropLastWhile { it.isBlank() }

      override fun toString() = command
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

  fun launch(rosPackage: String,
             launchFile: String,
             options: String = "",
             args: String = "") = object : Runnable {
    override fun run() {
      runCommandAndFetchOutput(env, shell.name, "-c", toString())
    }

    private val rosWorkspace: File = try {
      File(rosPackage).getContainingRosWorkspaceIfItExists()
    } catch (notFound: FileNotFoundException) {
      File("")
    }

    private val workspaceBuildSystem = if (buildSystem == catkin) {
      if (rosWorkspace.isDirectory && File("${rosWorkspace.absolutePath}${pathSeparator}.catkin_tools").isDirectory) {
        catkin_tools
      } else {
        catkin
      }
    } else {
      buildSystem
    }

    val rosDevelScriptPath = "${rosWorkspace.absolutePath}${pathSeparator}devel${pathSeparator}setup.$shell"

    override fun toString() =
      """echo 'ROS workspace directory: ${rosWorkspace.absolutePath}' &&
      cd ${rosWorkspace.absolutePath} &&
      ${workspaceBuildSystem.command} &&
      echo 'Sourcing $rosDevelScriptPath' &&
      . $rosDevelScriptPath &&
      echo 'Available nodes:' &&
      ${node.list} &&
      echo 'Available topics:' &&
      ${topic.list} &&
      echo 'Available services:' &&
      ${service.list} &&
      echo 'Available parameters:' &&
      ${param.list} &&
      ${this@Ros}launch $options $launchFile $args""".trimMargin()
  }

  val param = object : Listable(this) {
    override fun toString() = "${ros}param"
  }

  override fun toString() = command
}

fun runCommandAndFetchOutput(env: Map<String, String>, vararg commands: String): String {
  val proc = ProcessBuilder(*commands)
    .apply { environment().putAll(env) }
    .redirectOutput(ProcessBuilder.Redirect.PIPE)
    .start()

  proc.waitFor(10, TimeUnit.SECONDS)

  return if (proc.exitValue() != 0) "" else proc.inputStream.bufferedReader().readText()
}

fun runCommandInBackground(env: Map<String, String>, vararg commands: String) =
  ProcessBuilder(*commands).apply { environment().putAll(env) }.start()