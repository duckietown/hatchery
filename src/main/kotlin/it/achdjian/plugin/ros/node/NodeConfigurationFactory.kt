package it.achdjian.plugin.ros.node

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.execution.configurations.RunConfiguration
import com.intellij.execution.configurations.RunConfigurationSingletonPolicy
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.jetbrains.cidr.cpp.execution.CMakeAppRunConfiguration
import com.jetbrains.cidr.cpp.execution.CMakeBuildConfigurationHelper
import com.jetbrains.cidr.cpp.execution.CMakeRunConfigurationType
import it.achdjian.plugin.ros.ui.ICON_NODE


object IDs {
    const val ID = "ROS.rosNode"
    const val FACTORY = "ROS.factory.rosNode"
    const val DISPLAY_NAME = "ROS rosNode"
    const val DESCRIPTION = "Run ROS rosNode"
}


class NodeConfigurationFactory(private val configurationType: NodeConfigurationTypeCMake) : ConfigurationFactory(configurationType) {
    override fun createTemplateConfiguration(project: Project): RunConfiguration {
        return NodeConfiguration(project, configurationType.confFactory, "ROS")
    }
    override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
    override fun getId() = IDs.FACTORY
}

class NodeConfigurationTypeCMake :  CMakeRunConfigurationType(IDs.ID, IDs.FACTORY, IDs.DISPLAY_NAME,IDs.DESCRIPTION, ICON_NODE) {
    override fun createEditor(project: Project): SettingsEditor<out CMakeAppRunConfiguration> {
        return NodeRunEditorCMake(project, CMakeBuildConfigurationHelper(project))
    }

    override fun createRunConfiguration(project: Project, configurationFactory: ConfigurationFactory): CMakeAppRunConfiguration {
       return NodeConfiguration(project,confFactory, "ROS")
    }


    var confFactory = NodeConfigurationFactory(this)

}