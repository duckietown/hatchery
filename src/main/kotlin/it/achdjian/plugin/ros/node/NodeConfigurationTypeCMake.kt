package it.achdjian.plugin.ros.node

import com.intellij.execution.configurations.ConfigurationFactory
import com.intellij.execution.configurations.RunConfigurationSingletonPolicy
import com.intellij.openapi.project.Project
import com.jetbrains.cidr.cpp.execution.CMakeBuildConfigurationHelper
import com.jetbrains.cidr.cpp.execution.CMakeRunConfigurationType
import it.achdjian.plugin.ros.ui.ICON_NODE

object NodeConfigurationTypeCMake: CMakeRunConfigurationType("ROS.rosNode", "ROS.factory.rosNode", "ROS rosNode", "Run ROS rosNode", ICON_NODE) {
  override fun createEditor(project: Project) = NodeRunEditorCMake(project, CMakeBuildConfigurationHelper(project))
  override fun createRunConfiguration(project: Project, configurationFactory: ConfigurationFactory) =
    NodeConfigurationCMake(project, configurationFactory, "ROS")

  override fun getFactory() = object: ConfigurationFactory(this) {
    override fun createTemplateConfiguration(project: Project) = NodeConfigurationCMake(project, this, "ROS")
    override fun getSingletonPolicy() = RunConfigurationSingletonPolicy.SINGLE_INSTANCE_ONLY
    override fun getId() = "ROS.factory.rosNode"
  }
}