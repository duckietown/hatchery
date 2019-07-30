package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.LazyRunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigProducer : LazyRunConfigurationProducer<RosLaunchRunConfiguration>() {
  override fun getConfigurationFactory() = RosLaunchRunConfigFactory

  override fun isConfigurationFromContext(config: RosLaunchRunConfiguration,
                                          context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(config: RosLaunchRunConfiguration,
                                             context: ConfigurationContext,
                                             source: Ref<PsiElement>): Boolean {
    context.location?.virtualFile?.also { config.path = it } ?: return false

    return true
  }
}