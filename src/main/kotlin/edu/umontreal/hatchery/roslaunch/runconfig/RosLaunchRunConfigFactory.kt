package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.execution.configurations.RunConfigurationSingletonPolicy
import com.intellij.openapi.project.Project

object RosLaunchRunConfigFactory : ConfigurationFactory(RosLaunchRunConfigType) {
  override fun createTemplateConfiguration(project: Project) = RosLaunchRunConfiguration(project, RosLaunchRunConfigFactory, "")

  override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
}