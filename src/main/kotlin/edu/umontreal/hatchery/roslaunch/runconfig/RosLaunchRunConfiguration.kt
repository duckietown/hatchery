package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.filters.TextConsoleBuilderFactory
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.cli.RosCommandLineState

class RosLaunchRunConfiguration : LocatableConfigurationBase {
  var path = ""

  constructor(project: Project, name: String) :
    super(project, RosLaunchRunConfigurationFactory, name)

  override fun getConfigurationEditor() = RosLaunchSettingsEditor

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RosCommandLineState(environment, "roslaunch", path)
}