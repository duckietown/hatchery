import org.gradle.api.*
import org.gradle.api.tasks.*
import org.gradle.kotlin.dsl.*
import java.io.File

open class ROSTask : Exec() {
  val rosDistro = "kinetic"

  init {
    group = "My"
    description = "Prints a description of ${project.name}."
  }

  @TaskAction
  fun run() {

  }
}

//  val setupProjectEnv by tasks.creating(Exec::class) {
//    dependsOn(setupRosEnv)
//    executable = "bash"
//    val srcRoot = rosProjectRoot.walkTopDown()
//        .first { it.isDirectory && it.name == "catkin_ws" }.absolutePath
//    val develSetup = "$srcRoot/devel/setup.bash"
//    val commandString = """
//      catkin_make -C $srcRoot && \
//      chmod +x $develSetup && \
//      source $develSetup
//      """
//
//    args("-c", commandString)
//  }

/**
 * Declares a [ROSTask] named `hello`.
 */
fun Project.withRosTask() = tasks.register("rosTask", ROSTask::class) {
  executable = "bash"

  val weAreCurrentlyInRosEnv = System.getenv("ROS_DISTRO") != null
  val commandString = if (weAreCurrentlyInRosEnv) {
    "echo \"Using ROS_ROOT: \$ROS_ROOT\""
  } else {
    var rosSetupFile = File("/opt/ros/$rosDistro/setup.bash")
    if (rosSetupFile.exists()) {
      logger.info("Sourcing ROS $rosSetupFile")
    } else if (File("/opt/ros/").isDirectory) {
      rosSetupFile = File("/opt/ros/").walkTopDown().maxDepth(3).first { it.name == "setup.bash" }
      logger.warn("Unable to find default ROS distro ($rosDistro), using ${rosSetupFile.parentFile.path} instead")
    } else {
      throw GradleException("Unable to detect a usable setup.bash file in /opt/ros!")
    }

    val pluginDevArg = if (project.hasProperty("luginDev")) "-PluginDev" else ""
    "source $rosSetupFile && source gradlew runIde ${pluginDevArg}"
  }

  args("-c", commandString)

  val rosPython = "/opt/ros/$rosDistro/lib/python2.7/dist-packages"
// Try to set Python SDK default to ROS Python...
  val pythonPath = System.getenv()["PYTHONPATH"] ?: ""
  environment = mutableMapOf("PYTHONPATH" to "$rosPython:$pythonPath")
      .apply { putAll(System.getenv()) }
  logger.info("Python path: " + environment["PYTHONPATH"])

  doLast { if (!weAreCurrentlyInRosEnv) throw GradleException("Exiting IDE!") }
}
