package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.RunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigurationProducer :
  RunConfigurationProducer<RosLaunchRunConfiguration>(RosLaunchRunConfigurationType) {

  override fun isConfigurationFromContext(configuration: RosLaunchRunConfiguration,
                                          context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(configuration: RosLaunchRunConfiguration,
                                             context: ConfigurationContext,
                                             sourceElement: Ref<PsiElement>): Boolean {
    context.location?.virtualFile.also { file ->
      configuration.name = file?.nameWithoutExtension ?: return false
      configuration.rosLaunchFileName = file.name
      configuration.rosPackageName = file.parent?.parent?.name ?: ""
    } ?: return false

    return true
  }
}