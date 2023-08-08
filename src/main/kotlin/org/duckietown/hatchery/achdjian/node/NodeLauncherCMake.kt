package org.duckietown.hatchery.achdjian.node

import com.intellij.execution.ExecutionException
import com.intellij.execution.configurations.*
import com.intellij.execution.process.ProcessHandler
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.project.Project
import com.intellij.xdebugger.XDebugProcess
import com.intellij.xdebugger.XDebugSession
import com.jetbrains.cidr.cpp.execution.CMakeLauncher
import com.jetbrains.cidr.cpp.toolchains.CPPEnvironment
import com.jetbrains.cidr.execution.debugger.CidrDebugProcess
import org.duckietown.hatchery.achdjian.utils.getPackages
import java.io.File

class NodeLauncherCMake(private val nodeConfiguration: NodeConfiguration, private val prj: Project, environment: ExecutionEnvironment) :
        CMakeLauncher(environment, nodeConfiguration) {
    companion object {
        private val LOG = Logger.getInstance(NodeLauncherCMake::class.java)
    }

    @Throws(ExecutionException::class)
    override fun createProcess(state: CommandLineState): ProcessHandler {
        val packages = getPackages(project)
        val node = packages
                .filter { it.name == nodeConfiguration.rosPackageName }
                .flatMap { it.getNodes() }
                .firstOrNull { it.name == nodeConfiguration.rosNodeName }
//        node?.let {
//            configuration.executableData = ExecutableData(it.path.toString() )
//        }
        val process = super.createProcess(state)

        return process
    }

//    fun createCommandLine(state: CommandLineState, runFile: File, environment: CPPEnvironment, usePty: Boolean): GeneralCommandLine {
//
//
//        return super.createCommandLine(state, runFile, environment, usePty)
//    }


    @Throws(ExecutionException::class)
    override fun createDebugProcess(state: CommandLineState, session: XDebugSession): XDebugProcess {
        val packages = getPackages(project)
        val node = packages
                .filter { it.name == nodeConfiguration.rosPackageName }
                .flatMap { it.getNodes() }
                .firstOrNull { it.name == nodeConfiguration.rosNodeName }
//        node?.let {
//            configuration.executableData = ExecutableData(it.path.toString() )
//        }
        return super.createDebugProcess(state, session)
    }


    override fun getProject() = prj
}
