package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.Executor
import com.intellij.execution.configurations.LocatableConfigurationBase
import com.intellij.execution.configurations.RunConfiguration
import com.intellij.execution.configurations.RunProfileState
import com.intellij.execution.runners.ExecutionEnvironment
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project

class ROSLaunchRunConfiguration(project: Project, factory: ROSLaunchRunConfigurationFactory, name: String) : LocatableConfigurationBase(project, factory, name) {
    override fun getConfigurationEditor(): SettingsEditor<out RunConfiguration> {
        TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getState(executor: Executor, environment: ExecutionEnvironment): RunProfileState? {
        TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
    }
}