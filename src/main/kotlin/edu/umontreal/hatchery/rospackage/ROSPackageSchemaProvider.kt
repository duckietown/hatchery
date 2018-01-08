package edu.umontreal.hatchery.rospackage

import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.module.Module
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.PsiFile
import com.intellij.psi.PsiManager
import com.intellij.psi.xml.XmlFile
import com.intellij.xml.XmlSchemaProvider
import org.jetbrains.annotations.NonNls

class ROSPackageSchemaProvider : XmlSchemaProvider() {

    companion object {
        fun isPackageFile(name: String) = name.contains(".package")
    }

    override fun isAvailable(file: XmlFile): Boolean {
        return isPackageFile(file.name)
    }

    override fun getSchema(@NonNls url: String, module: Module?, baseFile: PsiFile): XmlFile? {
        return if (module != null && isPackageFile(baseFile.name)) module.let { getReference(it) } else null
    }

    private val LOG = Logger.getInstance(ROSPackageSchemaProvider::class.java)

    private fun getReference(module: Module): XmlFile? {
        val resource = ROSPackageSchemaProvider::class.java.getResource("rospackage.xsd")
        val fileByURL = VfsUtil.findFileByURL(resource)
        if (fileByURL == null) {
            LOG.error("xsd not found")
            return null
        }

        val psiFile = PsiManager.getInstance(module.project).findFile(fileByURL)
        return psiFile!!.copy() as XmlFile
    }
}
