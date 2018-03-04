package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.filters.TextConsoleBuilderFactory
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project

class RosLaunchRunConfiguration(project: Project, factory: RosLaunchRunConfigurationFactory, name: String) : LocatableConfigurationBase(project, factory, name) {
  override fun getConfigurationEditor() = RosLaunchSettingsEditor(project)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
      RosLaunchCommandLineState(environment).apply {
        consoleBuilder = TextConsoleBuilderFactory.getInstance().createBuilder(project)
      }
}