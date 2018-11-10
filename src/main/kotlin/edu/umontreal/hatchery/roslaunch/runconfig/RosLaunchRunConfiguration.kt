package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.RosInstall.rosSetupScript
import edu.umontreal.hatchery.RosInstall.shell
import edu.umontreal.hatchery.cli.RosCommandLineState
import java.io.File

class RosLaunchRunConfiguration : LocatableConfigurationBase {
  internal var rosPackageFile = File("")
  internal var rosLaunchFile = File("")

  internal var rosLaunchFileName
    get() = rosLaunchFile.name
    set(value) {
      rosLaunchFile = File(value)
    }

  internal var rosPackageName
    get() = rosPackageFile.name
    set(value) {
      rosPackageFile = File(value)
    }


  constructor(project: Project, name: String) :
    super(project, RosLaunchRunConfigurationFactory, name)

  override fun getConfigurationEditor() = RosLaunchSettingsEditor(project, rosPackageName, rosLaunchFileName)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RosCommandLineState(environment, "$shell", "-c",
      "source $rosSetupScript && " +
        "roslaunch $rosPackageName $rosLaunchFileName")
}