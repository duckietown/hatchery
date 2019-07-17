package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.project.Project
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
    element.setAttribute(ROS_LAUNCH_PATH, rosLaunchPath)
    element.setAttribute(ROS_PACKAGE_PATH, rosPackagePath)
    element.setAttribute(DESTINATION_ADDRESS, destinationAddress)
    element.setAttribute(DESTINATION_PATH, destinationPath)
    element.setAttribute(ROS_LAUNCH_OPTIONS, rosLaunchOptions)
    element.setAttribute(ROS_LAUNCH_ARGS, rosLaunchArgs)
  }

  override fun readExternal(element: Element) {
    super.readExternal(element)
    rosLaunchPath = element.getAttributeValue(ROS_LAUNCH_PATH)
    rosPackagePath = element.getAttributeValue(ROS_PACKAGE_PATH)
    destinationAddress = element.getAttributeValue(DESTINATION_ADDRESS)
    destinationPath = element.getAttributeValue(DESTINATION_PATH)
    rosLaunchOptions = element.getAttributeValue(ROS_LAUNCH_OPTIONS)
    rosLaunchArgs = element.getAttributeValue(ROS_LAUNCH_ARGS)
  }
//    RunAnythingRunProfileState(environment, RosConfig.settings.localRos.launch(rosPackagePath, rosLaunchPath).toString())
}