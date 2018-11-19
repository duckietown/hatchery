package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.openapi.project.Project

object RosLaunchRunConfigFactory: ConfigurationFactory(RosLaunchRunConfigType) {
  override fun createTemplateConfiguration(project: Project) =
    RosLaunchRunConfig(project, "rosLaunchRunConfiguration")

  override fun isConfigurationSingletonByDefault() = true
}