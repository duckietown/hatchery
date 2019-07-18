package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.LazyRunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigProducer : LazyRunConfigurationProducer<LaunchConfiguration>() {
  override fun getConfigurationFactory() = LaunchConfigurationFactory

  override fun isConfigurationFromContext(config: LaunchConfiguration,
                                          context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(config: LaunchConfiguration,
                                             context: ConfigurationContext,
                                             source: Ref<PsiElement>): Boolean {
    context.location?.virtualFile?.also { config.path = it } ?: return false

    return true
  }
}