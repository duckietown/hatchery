package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.openapi.project.Project

class RosLaunchRunConfigurationFactory(runConfigurationType: RosLaunchRunConfigurationType) : ConfigurationFactory(runConfigurationType) {
  override fun createTemplateConfiguration(project: Project) = RosLaunchRunConfiguration(project, this, "name")
  override fun isConfigurationSingletonByDefault() = true
}