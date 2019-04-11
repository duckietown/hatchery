package edu.umontreal.hatchery.rosmanifest

import com.intellij.openapi.module.Module
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.PsiFile
import com.intellij.psi.PsiManager
import com.intellij.psi.xml.XmlFile
import com.intellij.xml.XmlSchemaProvider
import org.jetbrains.annotations.NonNls

/*
 * Schema is taken from: http://www.ros.org/reps/rep-0140.html
 */

object RosManifestSchemaProvider : XmlSchemaProvider() {
  private const val schemaName = "rosmanifest.xsd"

  private val xsdFile by lazy { VfsUtil.findFileByURL(javaClass.getResource(schemaName))!! }

  private fun isPackageFile(name: String) = name.contains(".package")

  override fun isAvailable(file: XmlFile) = isPackageFile(file.name)

  override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile) = module?.let { getReference(it) }

  private fun getReference(module: Module) = PsiManager.getInstance(module.project).findFile(xsdFile) as XmlFile
}