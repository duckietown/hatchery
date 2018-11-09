package edu.umontreal.hatchery

import org.gradle.api.*
import org.gradle.api.tasks.*
import org.gradle.kotlin.dsl.*

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
  executable = "bash"

  val systemRosDistro = System.getenv("ROS_DISTRO")
  val commandString = if (null != systemRosDistro) {
    "echo \"Using ROS_ROOT: $systemRosDistro\""
  } else {
    val rosSetupScript = RosInstall.getRosSetupScript()
    val pluginDevArg = if (project.hasProperty("luginDev")) "-PluginDev" else ""
    "source $rosSetupScript && source gradlew runIde ${pluginDevArg}"
  }

  args("-c", commandString)

  doLast { if (null == systemRosDistro) throw GradleException("Exiting IDE!") }
}
