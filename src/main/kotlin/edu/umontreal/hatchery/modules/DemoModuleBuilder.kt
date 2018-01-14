package edu.umontreal.hatchery.modules

import com.intellij.ide.util.projectWizard.ModuleBuilder
import com.intellij.ide.util.projectWizard.WizardContext
import com.intellij.openapi.Disposable
import com.intellij.openapi.options.ConfigurationException
import com.intellij.openapi.roots.ModifiableRootModel


class DemoModuleBuilder : ModuleBuilder() {
  @Throws(ConfigurationException::class)
  override fun setupRootModel(model: ModifiableRootModel) = TODO("not yet")

  override fun getModuleType() = DemoModuleType

  override fun getCustomOptionsStep(context: WizardContext, parentDisposable: Disposable) = DemoModuleWizardStep()
}