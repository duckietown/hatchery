package org.duckietown.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.*
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import com.intellij.openapi.util.*
import com.intellij.openapi.vfs.*
import org.duckietown.hatchery.cli.RosCommandLineState
import org.duckietown.hatchery.settings.RosConfig
import org.jdom.Element

class RosLaunchRunConfiguration(project: Project, configurationFactory: ConfigurationFactory, targetName: String) :
  LocatableConfigurationBase<RunProfileState>(project, configurationFactory, targetName),
  RunConfigurationWithSuppressedDefaultDebugAction {
  companion object {
    const val PATH_TAG = "path"
    const val ROS_MASTER_ADDR_TAG = "ros_master_addr"
    const val ROS_MASTER_PORT_TAG = "ros_master_port"
    const val SCREEN_TAG = "screen"
    const val LOG_TAG = "log"
    const val WAIT_TAG = "wait"
    const val VERBOSE_TAG = "verbose"
    const val LOG_LEVEL_TAG = "log_level"
  }

  var path: VirtualFile? = null
  var rosMasterAddr = RosConfig.settings.localRos.env["ROS_MASTER_URI"] ?: "localhost"
  var rosMasterPort = 11311
  var screen = false
  var log = false
  var wait = false
  var verbose = false
  var logLevel = "info"

  override fun getConfigurationEditor() = RosLaunchRunConfigEditor(project)

  override fun getState(executor: Executor, exeEnv: ExecutionEnvironment) =
    RosCommandLineState(exeEnv, RosConfig.settings.localRos.shell.name, "-c",
      RosConfig.settings.localRos.launch(launchFile = path?.path ?: "",
        customEnv = mapOf("ROS_MASTER_URI" to "http://$rosMasterAddr:$rosMasterPort",
          "PYTHONUNBUFFERED" to "1"), options = getParameters(), args = "").toString())

  private fun getParameters() = "--master-logger-level=$logLevel" +
    if (verbose) "-v " else "" +
      if (wait) "--wait " else "" +
        if (screen) "--screen " else "" +
          if (log) "--log" else ""

  @Throws(InvalidDataException::class)
  override fun readExternal(parentElement: Element) {
    with(parentElement) {
      super.readExternal(this)
      getAttributeValue(PATH_TAG)?.let {
        path = VirtualFileManager.getInstance().findFileByUrl(it)
      }
      getAttributeValue(ROS_MASTER_ADDR_TAG)?.let { rosMasterAddr = it }
      getAttributeValue(ROS_MASTER_PORT_TAG)?.let { rosMasterPort = it.toInt() }
      getAttributeValue(SCREEN_TAG)?.let { screen = it.toBoolean() }
      getAttributeValue(LOG_TAG)?.let { log = it.toBoolean() }
      getAttributeValue(WAIT_TAG)?.let { wait = it.toBoolean() }
      getAttributeValue(VERBOSE_TAG)?.let { verbose = it.toBoolean() }
      getAttributeValue(LOG_LEVEL_TAG)?.let { logLevel = it }
    }
  }

  @Throws(WriteExternalException::class)
  override fun writeExternal(parentElement: Element) {
    super.writeExternal(parentElement)
    path?.let { parentElement.setAttribute(PATH_TAG, it.url) }
    parentElement.setAttribute(ROS_MASTER_ADDR_TAG, rosMasterAddr)
    parentElement.setAttribute(ROS_MASTER_PORT_TAG, rosMasterPort.toString())
    parentElement.setAttribute(SCREEN_TAG, screen.toString())
    parentElement.setAttribute(LOG_TAG, log.toString())
    parentElement.setAttribute(WAIT_TAG, wait.toString())
    parentElement.setAttribute(VERBOSE_TAG, verbose.toString())
    parentElement.setAttribute(LOG_LEVEL_TAG, logLevel)
  }
}