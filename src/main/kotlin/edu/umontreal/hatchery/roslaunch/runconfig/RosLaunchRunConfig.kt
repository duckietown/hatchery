package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.ide.actions.runAnything.execution.RunAnythingRunProfileState
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.ros.Ros
import edu.umontreal.hatchery.settings.RosConfig

class RosLaunchRunConfig: LocatableConfigurationBase<RunProfileState> {
  constructor(project: Project, name: String):
    super(project, RosLaunchRunConfigFactory, name)

  internal var remoteAddress = RosConfig.settings.remoteAddress
  internal var remoteRosPath = RosConfig.settings.remoteRosPath

  var rosLaunchPath = ""
  var rosPackagePath = ""

  override fun getConfigurationEditor() =
    RosLaunchSettingsEditor(project, rosPackagePath, rosLaunchPath)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RunAnythingRunProfileState(environment, Ros().launch(rosPackagePath, rosLaunchPath).toString())
}