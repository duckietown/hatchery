package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.*
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project

class RosLaunchRunConfiguration(project: Project, factory: RosLaunchRunConfigurationFactory, name: String) : LocatableConfigurationBase(project, factory, name) {
  override fun getConfigurationEditor(): SettingsEditor<out RunConfiguration> {
    TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
  }

  override fun getState(executor: Executor, environment: ExecutionEnvironment): RunProfileState? {
    TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
  }
}