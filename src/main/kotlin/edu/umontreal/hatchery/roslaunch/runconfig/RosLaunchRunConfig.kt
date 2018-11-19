package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.cli.RosCommandLineState
import edu.umontreal.hatchery.ros.getContainingRosWorkspaceIfItExists
import edu.umontreal.hatchery.ros.rosDevelScriptPathRel
import edu.umontreal.hatchery.ros.rosSetupScript
import edu.umontreal.hatchery.ros.shell
import edu.umontreal.hatchery.settings.RosConfig
import java.io.File

class RosLaunchRunConfig: LocatableConfigurationBase {
  constructor(project: Project, name: String):
    super(project, RosLaunchRunConfigFactory, name)

  internal var rosPackageFile = File("")
  internal var rosLaunchFile = File("")
  internal var rosWorkspace = File("")
    get() = rosPackageFile.getContainingRosWorkspaceIfItExists()

  internal var runCommand =
    "echo Sourcing $rosSetupScript && source $rosSetupScript && " +
      "echo 'ROS workspace directory: ${rosWorkspace.absolutePath}' && " +
      "cd ${rosWorkspace.absolutePath} && catkin_make && " +
      "echo 'Sourcing ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel && " +
      "source ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel && " +
      "${RosConfig.settings.localRunCommand} $rosPackageName $rosLaunchFileName"

  internal var remoteAddress = RosConfig.settings.remoteAddress
  internal var remoteRosPath = RosConfig.settings.remoteRosPath

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

  override fun getConfigurationEditor() = RosLaunchSettingsEditor(project, rosPackageName, rosLaunchFileName)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RosCommandLineState(environment, "$shell", "-c", runCommand)
}