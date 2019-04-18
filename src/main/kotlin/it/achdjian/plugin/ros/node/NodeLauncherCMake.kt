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

class NodeLauncherCMake(private val nodeConfiguration: NodeConfigurationCMake, private val prj: Project, environment: ExecutionEnvironment) :
        CMakeLauncher(environment, nodeConfiguration) {
    companion object {
        private val LOG = Logger.getInstance(NodeLauncherCMake::class.java)
    }

    @Throws(ExecutionException::class)
    override fun createProcess(state: CommandLineState): ProcessHandler {
        val packages = getPackages(project)
        val node = packages
                .filter { it.name==nodeConfiguration.rosPackageName }
                .flatMap { it.getNodes() }
                .firstOrNull { it.name == nodeConfiguration.rosNodeName }
        node?.let {
            myConfiguration.executableData = ExecutableData(it.path.toString() )
        }
        return super.createProcess(state)
    }

    @Throws(ExecutionException::class)
    override fun createDebugProcess(state: CommandLineState, session: XDebugSession): CidrDebugProcess {
        val packages = getPackages(project)
        val node = packages
                .filter { it.name==nodeConfiguration.rosPackageName }
                .flatMap { it.getNodes() }
                .firstOrNull { it.name == nodeConfiguration.rosNodeName }
        node?.let {
            myConfiguration.executableData = ExecutableData(it.path.toString() )
        }
        return super.createDebugProcess(state,session)
    }



    override fun getProject() = prj
}