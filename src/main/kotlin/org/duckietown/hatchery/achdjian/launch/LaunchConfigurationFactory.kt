package org.duckietown.hatchery.achdjian.launch

import com.intellij.execution.configurations.*
import com.intellij.openapi.project.Project
import org.duckietown.hatchery.achdjian.ui.ICON_LAUNCH

object IDs {
    val ID = "ROS.launch"
    val FACTORY = "ROS.launch.factory"
    val DISPLAY_NAME = "ROS launch"
    val DESCRIPTION = "Run ROS launch file"
}


class LaunchConfigurationFactory(val configurationType: LaunchConfigurationType) : ConfigurationFactory(configurationType) {
    override fun createTemplateConfiguration(project: Project) = LaunchConfiguration(project, configurationType.confFactory, "")
    override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
    override fun getId() = IDs.FACTORY

}

class LaunchConfigurationType : ConfigurationTypeBase(IDs.ID, IDs.DISPLAY_NAME, IDs.DESCRIPTION, ICON_LAUNCH) {
    var confFactory = LaunchConfigurationFactory(this)

    init {
        addFactory(object : ConfigurationFactory(this) {
            override fun createTemplateConfiguration(project: Project): RunConfiguration =
                    LaunchConfiguration(project, this, IDs.FACTORY)

        })
    }
}