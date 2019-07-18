package it.achdjian.plugin.ros.launch

import com.intellij.execution.configurations.CommandLineState
import com.intellij.execution.configurations.GeneralCommandLine
import com.intellij.execution.process.KillableColoredProcessHandler
import com.intellij.execution.process.NopProcessHandler
import com.intellij.execution.runners.ExecutionEnvironment
import edu.umontreal.hatchery.roslaunch.runconfig.LaunchConfiguration
import it.achdjian.plugin.ros.data.RosVersionImpl
import it.achdjian.plugin.ros.utils.getBaseDir
import it.achdjian.plugin.ros.utils.getEnvironmentVariables
import it.achdjian.plugin.ros.utils.getVersion

class LaunchLauncher(private val launchConfiguration: LaunchConfiguration, environment: ExecutionEnvironment) : CommandLineState(environment) {
  override fun startProcess() = launchConfiguration.path?.let { launchFile ->
    val rosVersion: RosVersionImpl? = getVersion(environment.project)
    rosVersion?.let {
      val environmentVariables = getEnvironmentVariables(environment.project, it.env)
      val cmdLine = GeneralCommandLine(it.rosLaunch).apply {
        if (launchConfiguration.verbose) addParameter("-v")
        if (launchConfiguration.wait) addParameter("--wait")
        if (launchConfiguration.screen) addParameter("--screen")
        if (launchConfiguration.log) addParameter("--log")
          addParameter("--master-logger-level=${launchConfiguration.logLevel}")
        addParameter(launchFile.path)
        withWorkDirectory(getBaseDir(this@LaunchLauncher.environment.project)?.path)
        withEnvironment(environmentVariables)
        withEnvironment("PYTHONUNBUFFERED", "1")
        val rosMasterUri = "http://${launchConfiguration.rosMasterAddr}:${launchConfiguration.rosMasterPort}"
        withEnvironment("ROS_MASTER_URI", rosMasterUri)
      }
      KillableColoredProcessHandler(cmdLine)
    } ?: NopProcessHandler()
  } ?: NopProcessHandler()
}