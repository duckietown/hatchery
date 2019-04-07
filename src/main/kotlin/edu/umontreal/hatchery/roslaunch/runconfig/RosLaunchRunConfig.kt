package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.cli.RosCommandLineState
import edu.umontreal.hatchery.settings.RosConfig

class RosLaunchRunConfig : LocatableConfigurationBase<RunProfileState> {
  constructor(project: Project, name: String) :
    super(project, RosLaunchRunConfigFactory, name)

  var rosLaunchPath = ""
  var rosPackagePath = ""

  var destinationAddress = ""
  var destinationPath = ""
  var rosLaunchOptions: String = RosConfig.settings.defaultRosLaunchOptions
  var rosLaunchArgs: String = ""

  override fun getConfigurationEditor() =
    RosLaunchSettingsEditor(project, rosPackagePath, rosLaunchPath)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RosCommandLineState(environment, RosConfig.settings.localRos.shell.name, "-c",
      RosConfig.settings.localRos.launch(
        rosPackagePath,
        rosLaunchPath,
        rosLaunchOptions,
        rosLaunchArgs).toString()
    )
//    RunAnythingRunProfileState(environment, RosConfig.settings.localRos.launch(rosPackagePath, rosLaunchPath).toString())
}