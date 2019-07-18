package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
import com.intellij.openapi.util.JDOMExternalizerUtil.readField
import com.intellij.openapi.util.JDOMExternalizerUtil.writeField
import edu.umontreal.hatchery.cli.RosCommandLineState
import edu.umontreal.hatchery.settings.RosConfig
import org.jdom.Element

class RosLaunchRunConfig : LocatableConfigurationBase<RunProfileState> {
  constructor(project: Project, name: String) :
    super(project, RosLaunchRunConfigFactory, name)

  var rosLaunchPath = ""
  val ROS_LAUNCH_PATH = "ros-launch-path"
  var rosPackagePath = ""
  val ROS_PACKAGE_PATH = "ros-package-path"

  var destinationAddress = ""
  val DESTINATION_ADDRESS = "destination-address"
  var destinationPath = ""
  val DESTINATION_PATH = "destination-path"
  var rosLaunchOptions: String = RosConfig.settings.defaultRosLaunchOptions
  val ROS_LAUNCH_OPTIONS = "ros-launch-options"
  var rosLaunchArgs: String = ""
  val ROS_LAUNCH_ARGS = "ros-launch-args"


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

  override fun writeExternal(element: Element) {
    super.writeExternal(element)
    writeField(element, ROS_LAUNCH_PATH, rosLaunchPath)
    writeField(element, ROS_PACKAGE_PATH, rosPackagePath)
    writeField(element, DESTINATION_ADDRESS, destinationAddress)
    writeField(element, DESTINATION_PATH, destinationPath)
    writeField(element, ROS_LAUNCH_OPTIONS, rosLaunchOptions)
    writeField(element, ROS_LAUNCH_ARGS, rosLaunchArgs)
  }

  override fun readExternal(element: Element) {
    super.readExternal(element)
    rosLaunchPath = readField(element, ROS_LAUNCH_PATH) ?: ""
    rosLaunchPath = readField(element, ROS_PACKAGE_PATH) ?: ""
    rosLaunchPath = readField(element, DESTINATION_ADDRESS) ?: ""
    rosLaunchPath = readField(element, DESTINATION_PATH) ?: ""
    rosLaunchPath = readField(element, ROS_LAUNCH_OPTIONS) ?: ""
    rosLaunchPath = readField(element, ROS_LAUNCH_ARGS) ?: ""
  }
//    RunAnythingRunProfileState(environment, RosConfig.settings.localRos.launch(rosPackagePath, rosLaunchPath).toString())
}