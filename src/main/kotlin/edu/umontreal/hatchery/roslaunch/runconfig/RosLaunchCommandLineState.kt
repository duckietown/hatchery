package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.CommandLineState
import com.intellij.execution.configurations.GeneralCommandLine
import com.intellij.execution.process.ColoredProcessHandler
import com.intellij.execution.process.ProcessTerminatedListener
import com.intellij.execution.runners.ExecutionEnvironment

class RosLaunchCommandLineState(executionEnvironment: ExecutionEnvironment,
                                private val command: String = "echo",
                                private val commandLine: GeneralCommandLine = GeneralCommandLine(command)) : CommandLineState(executionEnvironment) {
  override fun startProcess() = ColoredProcessHandler(commandLine).apply { ProcessTerminatedListener.attach(this) }
}