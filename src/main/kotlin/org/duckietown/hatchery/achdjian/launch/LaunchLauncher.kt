package org.duckietown.hatchery.achdjian.launch

import com.intellij.execution.configurations.*
import com.intellij.execution.process.*
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.diagnostic.Logger
import org.duckietown.hatchery.achdjian.utils.*

class LaunchLauncher(private val launchConfiguration: LaunchConfiguration, environment: ExecutionEnvironment) : CommandLineState(environment) {
    companion object {
        private val LOG = Logger.getInstance(LaunchLauncher::class.java)
    }

    override fun startProcess() = launchConfiguration.path?.let { launchFile ->
        val rosVersion = getVersion(environment.project)
        rosVersion?.let {
            val environmentVariables = getEnvironmentVariables(environment.project, it.env)
            val cmdLine = GeneralCommandLine(it.rosLaunch)
            if (launchConfiguration.verbose) {
                cmdLine.addParameter("-v")
            }
            if (launchConfiguration.wait) {
                cmdLine.addParameter("--wait")
            }
            if (launchConfiguration.screen) {
                cmdLine.addParameter("--screen")
            }
//            if (launchConfiguration.log) {
//                cmdLine.addParameter("--log")
//            }
//            cmdLine.addParameter("--master-logger-level=${launchConfiguration.logLevel}")
            cmdLine.addParameter(launchFile.path)
            cmdLine.withWorkDirectory(getBaseDir(environment.project)?.path)
            cmdLine.withEnvironment(environmentVariables)
            cmdLine.withEnvironment("PYTHONUNBUFFERED", "1")
            val rosMasterUri = "http://${launchConfiguration.rosMasterAddr}:${launchConfiguration.rosMasterPort}"
            cmdLine.withEnvironment("ROS_MASTER_URI", rosMasterUri)
            val handler = KillableColoredProcessHandler(cmdLine)
            handler
        } ?: NopProcessHandler()
    } ?: NopProcessHandler()
}