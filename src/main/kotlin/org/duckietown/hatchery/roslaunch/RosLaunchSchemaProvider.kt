package org.duckietown.hatchery.roslaunch

import com.intellij.openapi.module.Module
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.*
import com.intellij.psi.xml.XmlFile
import com.intellij.xml.XmlSchemaProvider
import org.jetbrains.annotations.NonNls

/*
 * Schema from: https://github.com/ros/ros_comm/issues/455#issuecomment-355625591
 */

object RosLaunchSchemaProvider : XmlSchemaProvider() {
  private const val schemaName = "roslaunch.xsd"

  val xsdFile by lazy { VfsUtil.findFileByURL(javaClass.getResource(schemaName))!! }

  private fun isLaunchFile(name: String) = name.contains(".launch")

  override fun isAvailable(file: XmlFile) = isLaunchFile(file.name)

  override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile) = module?.let { getReference(it) }

  private fun getReference(module: Module) = PsiManager.getInstance(module.project).findFile(xsdFile) as XmlFile
}
