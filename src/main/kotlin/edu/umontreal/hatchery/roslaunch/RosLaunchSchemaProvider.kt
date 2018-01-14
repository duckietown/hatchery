package edu.umontreal.hatchery.roslaunch

import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.module.Module
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.PsiFile
import com.intellij.psi.PsiManager
import com.intellij.psi.xml.XmlFile
import com.intellij.xml.XmlSchemaProvider
import edu.umontreal.hatchery.rospackage.RosPackageSchemaProvider
import org.jetbrains.annotations.NonNls

/*
 * Schema from: https://github.com/ros/ros_comm/issues/455#issuecomment-355625591
 */

class RosLaunchSchemaProvider : XmlSchemaProvider() {
  private fun isLaunchFile(name: String) = name.contains(".launch")

  override fun isAvailable(file: XmlFile) = isLaunchFile(file.name)

  override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile): XmlFile? {
    return if (module != null && isLaunchFile(baseFile.name)) module.let { getReference(it) } else null
  }

  private val LOG = Logger.getInstance(RosLaunchSchemaProvider::class.java)

  private fun getReference(module: Module): XmlFile? {
    val xsdFile = VfsUtil.findFileByURL(javaClass.getResource("roslaunch.xsd"))
    if (xsdFile == null) {
      LOG.error("xsd not found")
      return null
    }

    return PsiManager.getInstance(module.project).findFile(xsdFile) as XmlFile
  }
}
