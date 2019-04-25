package it.achdjian.plugin.ros.node

import com.intellij.execution.ExecutionException
import com.intellij.execution.configurations.CommandLineState
import com.intellij.execution.process.ProcessHandler
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.project.Project
import com.intellij.xdebugger.XDebugSession
import com.jetbrains.cidr.cpp.execution.CMakeLauncher
import com.jetbrains.cidr.execution.ExecutableData
import com.jetbrains.cidr.execution.debugger.CidrDebugProcess
import it.achdjian.plugin.ros.utils.getPackages

class NodeLauncherCMake(
  private val nodeConfiguration: NodeConfigurationCMake,
  private val prj: Project, environment: ExecutionEnvironment
) : CMakeLauncher(environment, nodeConfiguration) {
  @Throws(ExecutionException::class)
  override fun createProcess(state: CommandLineState) =
    prepareNodeConfig().run { super.createProcess(state) }

  @Throws(ExecutionException::class)
  override fun createDebugProcess(state: CommandLineState, session: XDebugSession) =
    prepareNodeConfig().run { super.createDebugProcess(state, session) }

  private fun prepareNodeConfig() =
    getPackages(project)
      .filter { it.name == nodeConfiguration.rosPackageName }
      .flatMap { it.getNodes() }
      .firstOrNull { it.name == nodeConfiguration.rosNodeName }
      ?.also {
        nodeConfiguration.executableData = ExecutableData(it.path.toString())
      }

  override fun getProject() = prj
}