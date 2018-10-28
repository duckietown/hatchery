package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.openapi.project.Project

object RosLaunchRunConfigurationFactory : ConfigurationFactory(RosLaunchRunConfigurationType) {
  override fun createTemplateConfiguration(project: Project) =
    RosLaunchRunConfiguration(project, "rosLaunchRunConfiguration")

  override fun isConfigurationSingletonByDefault() = true
}