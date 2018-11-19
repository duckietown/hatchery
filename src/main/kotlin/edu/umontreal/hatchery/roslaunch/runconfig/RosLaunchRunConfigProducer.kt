package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.RunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigProducer:
  RunConfigurationProducer<RosLaunchRunConfig>(RosLaunchRunConfigType) {

  override fun isConfigurationFromContext(config: RosLaunchRunConfig,
                                          context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(config: RosLaunchRunConfig,
                                             context: ConfigurationContext,
                                             source: Ref<PsiElement>): Boolean {
    context.location?.virtualFile.also { file ->
      config.name = file?.nameWithoutExtension ?: return false
      config.rosLaunchFileName = file.canonicalPath
      config.rosPackageName = file.parent?.parent?.canonicalPath ?: ""
    } ?: return false

    return true
  }
}