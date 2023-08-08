package org.duckietown.hatchery.modules

import com.intellij.icons.AllIcons
import com.intellij.openapi.module.ModuleType

class DemoModuleType : ModuleType<DemoModuleBuilder>("DEMO_MODULE_TYPE") {
  override fun createModuleBuilder() = DemoModuleBuilder()

  override fun getName() = "Demo Module Type"

  override fun getDescription() = "Demo Module Type"

  override fun getNodeIcon(b: Boolean) = AllIcons.General.Information
}
