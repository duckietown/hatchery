package edu.umontreal.hatchery

import edu.umontreal.hatchery.RosInstall.defaultShell
import org.gradle.api.GradleException
import org.gradle.api.Project
import org.gradle.api.tasks.Exec
import org.gradle.api.tasks.TaskAction
import org.gradle.kotlin.dsl.register

open class ROSTask : Exec() {

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
 *
 * http://wiki.ros.org/ROS/EnvironmentVariables
 */

fun Project.withRosTask() = tasks.register("rosTask", ROSTask::class) {
  executable = "$defaultShell"

  val systemRosDistro = System.getenv("ROS_DISTRO")
  val commandString = if (null != systemRosDistro) {
    "echo \"Using ROS_ROOT: $systemRosDistro\""
  } else {
    val rosSetupScript = RosInstall.rosSetupScript
    val pluginDevArg = if (project.hasProperty("luginDev")) "-PluginDev" else ""
    "source $rosSetupScript && source gradlew runIde ${pluginDevArg}"
  }

  args("-c", commandString)

  doLast { if (null == systemRosDistro) throw GradleException("Exiting IDE!") }
}
