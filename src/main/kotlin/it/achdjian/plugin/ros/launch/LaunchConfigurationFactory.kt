package it.achdjian.plugin.ros.launch

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.execution.configurations.RunConfigurationSingletonPolicy
import com.intellij.openapi.project.Project
import edu.umontreal.hatchery.roslaunch.runconfig.LaunchConfiguration
import edu.umontreal.hatchery.roslaunch.runconfig.LaunchConfigurationFactory
import edu.umontreal.hatchery.roslaunch.runconfig.RosLaunchRunConfigType

object LaunchConfigurationFactory : ConfigurationFactory(RosLaunchRunConfigType) {
  override fun createTemplateConfiguration(project: Project) = LaunchConfiguration(project, LaunchConfigurationFactory, "")

  override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
}