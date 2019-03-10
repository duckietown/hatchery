package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.cli.RosCommandLineState
import edu.umontreal.hatchery.ros.getContainingRosWorkspaceIfItExists
import edu.umontreal.hatchery.ros.rosDevelScriptPathRel
import edu.umontreal.hatchery.ros.rosSetupScript
import edu.umontreal.hatchery.ros.shell
import edu.umontreal.hatchery.settings.RosConfig
import java.io.File
import java.io.FileNotFoundException

class RosLaunchRunConfig: LocatableConfigurationBase<RunProfileState> {
  private val logger = Logger.getInstance(RosLaunchRunConfig::class.java)
  constructor(project: Project, name: String):
    super(project, RosLaunchRunConfigFactory, name)

  private var rosPackageFile = File("")
  private var rosLaunchFile = File("")
  private val rosWorkspace: File
    get() = try {
      rosPackageFile.getContainingRosWorkspaceIfItExists()
    } catch (notFound: FileNotFoundException) {
      logger.error("Unable to find parent ROS workspace! $notFound")
      File("")
    }

  internal var runCommand = ""
    get() = if (field.isNotEmpty()) field
    else """echo Sourcing $rosSetupScript && source $rosSetupScript &&
      echo ROS workspace directory: ${rosWorkspace.absolutePath} &&
      cd ${rosWorkspace.absolutePath} && catkin_make &&
      echo Sourcing ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel &&
      source ${rosWorkspace.absolutePath}/$rosDevelScriptPathRel &&
      echo Available nodes: && rosnode list &&
      echo Available topics: && rostopic list &&
      echo Available services: && rosservice list &&
      echo Available parameters: && rosparam list &&
      ${RosConfig.settings.localRunCommand} $rosPackageName $rosLaunchFileName""".trimMargin()

  internal var remoteAddress = RosConfig.settings.remoteAddress
  internal var remoteRosPath = RosConfig.settings.remoteRosPath

  internal var rosLaunchPath
    get() = rosLaunchFile.path
    set(value) {
      rosLaunchFile = File(value)
    }

  internal var rosPackagePath
    get() = rosPackageFile.path
    set(value) {
      rosPackageFile = File(value)
    }

  internal val rosPackageName
    get() = rosPackageFile.name
  internal val rosLaunchFileName
    get() = rosLaunchFile.name

  override fun getConfigurationEditor() =
    RosLaunchSettingsEditor(project, rosPackagePath, rosLaunchPath)

  override fun getState(executor: Executor, environment: ExecutionEnvironment) =
    RosCommandLineState(environment, "$shell", "-c", runCommand)
}