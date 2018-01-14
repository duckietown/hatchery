package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.openapi.project.Project

class ROSLaunchRunConfigurationFactory(runConfigurationType: ROSLaunchRunConfigurationType) : ConfigurationFactory(runConfigurationType) {
    override fun createTemplateConfiguration(project: Project) = ROSLaunchRunConfiguration(project, this, "name")
    override fun isConfigurationSingletonByDefault() = true
}