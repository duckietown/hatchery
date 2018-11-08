package edu.umontreal.hatchery.cli

import com.intellij.execution.configurations.CommandLineState
import com.intellij.execution.configurations.GeneralCommandLine
import com.intellij.execution.configurations.GeneralCommandLine.ParentEnvironmentType.SYSTEM
import com.intellij.execution.filters.TextConsoleBuilderFactory
import com.intellij.execution.process.ColoredProcessHandler
import com.intellij.execution.process.ProcessTerminatedListener
import com.intellij.execution.runners.ExecutionEnvironment

class RosCommandLineState : CommandLineState {
  private val commands: Array<out String>
  val commandLine: GeneralCommandLine

  constructor(env: ExecutionEnvironment, vararg commands: String) : super(env) {
    this.commands = commands
    this.commandLine = createCommandLine()
    consoleBuilder = TextConsoleBuilderFactory.getInstance().createBuilder(env.project)
  }

  private fun createCommandLine() =
    GeneralCommandLine(*commands)
      .withEnvironment(System.getenv())
      .withParentEnvironmentType(SYSTEM)

  override fun startProcess() =
    ColoredProcessHandler(commandLine).apply { ProcessTerminatedListener.attach(this) }
}