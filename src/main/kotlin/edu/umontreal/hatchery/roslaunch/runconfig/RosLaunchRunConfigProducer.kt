package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.actions.ConfigurationContext
import com.intellij.execution.actions.LazyRunConfigurationProducer
import com.intellij.openapi.util.Ref
import com.intellij.psi.PsiElement

object RosLaunchRunConfigProducer: LazyRunConfigurationProducer<RosLaunchRunConfig>() {
  override fun getConfigurationFactory() = RosLaunchRunConfigFactory

  override fun isConfigurationFromContext(config: RosLaunchRunConfig,
                                          context: ConfigurationContext) =
    context.location?.virtualFile?.extension == ".launch"

  override fun setupConfigurationFromContext(config: RosLaunchRunConfig,
                                             context: ConfigurationContext,
                                             source: Ref<PsiElement>): Boolean {
    context.location?.virtualFile.also { file ->
      config.name = file?.nameWithoutExtension ?: return false
      config.rosLaunchPath = file.canonicalPath ?: ""
      config.rosPackagePath = file.parent?.parent?.canonicalPath ?: ""
    } ?: return false

    return true
  }
}