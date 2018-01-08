package edu.umontreal.hatchery.roslaunch

import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.module.Module
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.PsiFile
import com.intellij.psi.PsiManager
import com.intellij.psi.xml.XmlFile
import com.intellij.xml.XmlSchemaProvider
import org.jetbrains.annotations.NonNls

class ROSLaunchSchemaProvider : XmlSchemaProvider() {
    companion object {
        fun isLaunchFile(name: String) = name.contains(".launch")
    }

    override fun isAvailable(file: XmlFile): Boolean {
        return isLaunchFile(file.name)
    }

    override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile): XmlFile? {
        return if (module != null && isLaunchFile(baseFile.name)) module.let { getReference(it) } else null
    }

    private val LOG = Logger.getInstance(ROSLaunchSchemaProvider::class.java)

    private fun getReference(module: Module): XmlFile? {
        val resource = ROSLaunchSchemaProvider::class.java.getResource("roslaunch.xsd")
        val fileByURL = VfsUtil.findFileByURL(resource)
        if (fileByURL == null) {
            LOG.error("xsd not found")
            return null
        }

        val psiFile = PsiManager.getInstance(module.project).findFile(fileByURL)
        return psiFile!!.copy() as XmlFile
    }
}
