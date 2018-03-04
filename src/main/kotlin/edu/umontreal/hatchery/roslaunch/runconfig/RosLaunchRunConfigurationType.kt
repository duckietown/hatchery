package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationType
import edu.umontreal.hatchery.filesystem.Icons

object RosLaunchRunConfigurationType : ConfigurationType {
  override fun getIcon() = Icons.ros_launch

  override fun getConfigurationTypeDescription() = "roslaunch_runconfig_description"

  override fun getId() = "roslaunch_runconfig_id"

  override fun getDisplayName() = "roslaunch_runconfig_display_name"

  override fun getConfigurationFactories() = arrayOf(RosLaunchRunConfigurationFactory(this))
}
