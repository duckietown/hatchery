package org.duckietown.hatchery.roslaunch.runconfig

import com.intellij.execution.configurations.*
import com.intellij.openapi.project.Project

object RosLaunchRunConfigFactory : ConfigurationFactory(RosLaunchRunConfigType) {
  override fun createTemplateConfiguration(project: Project) = RosLaunchRunConfiguration(project, RosLaunchRunConfigFactory, "")

  override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
}