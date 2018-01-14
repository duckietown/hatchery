package edu.umontreal.hatchery.rospackage

import com.intellij.openapi.diagnostic.Logger
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

class RosPackageSchemaProvider : XmlSchemaProvider() {
  private val LOG = Logger.getInstance(RosPackageSchemaProvider::class.java)

  private fun isPackageFile(name: String) = name.contains(".package")

  override fun isAvailable(file: XmlFile) = isPackageFile(file.name)

  override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile) =
      if (module != null && isPackageFile(baseFile.name)) module.let { getReference(it) } else null

  private fun getReference(module: Module): XmlFile? {
    val xsdFile = VfsUtil.findFileByURL(javaClass.getResource("rospackage.xsd"))
    if (xsdFile == null) {
      LOG.error("xsd not found")
      return null
    }

    return PsiManager.getInstance(module.project).findFile(xsdFile) as XmlFile
  }
}
