package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationType
import edu.umontreal.hatchery.filesystem.Icons

class ROSLaunchRunConfigurationType : ConfigurationType {
    override fun getIcon() = Icons.ros_launch

    override fun getConfigurationTypeDescription() = "roslaunch_description"

    override fun getId() = "roslaunch_id"

    override fun getDisplayName() = "roslaunch_display_name"

    override fun getConfigurationFactories() = arrayOf(ROSLaunchRunConfigurationFactory(this))
}
