package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.RunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigurationProducer : RunConfigurationProducer<RosLaunchRunConfiguration>(RosLaunchRunConfigurationType) {
  override fun isConfigurationFromContext(configuration: RosLaunchRunConfiguration, context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(configuration: RosLaunchRunConfiguration, context: ConfigurationContext, sourceElement: Ref<PsiElement>): Boolean {
    configuration.name = context.location?.virtualFile?.name ?: return false

    return true
  }
}