package edu.umontreal.hatchery

import com.intellij.openapi.project.Project
import com.intellij.openapi.roots.ProjectRootManager
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.PsiFileSystemItem
import com.intellij.psi.PsiManager
import com.intellij.psi.impl.source.resolve.reference.impl.providers.FileReferenceHelper
import com.intellij.util.xml.DomManager
import com.intellij.psi.xml.XmlFile
import com.intellij.util.xml.DomFileElement
import com.intellij.psi.PsiFile



/**
 * Provides resource roots to be used as contexts in FileReferenceSet during the XIncludes resolving.
 */
class ROSFileReferenceHelper: FileReferenceHelper() {
    override fun isMine(project: Project, file: VirtualFile) =
        file.fileType === ROSPackageFileFactory.FileType &&
                isPackageXml(PsiManager.getInstance(project).findFile(file)!!)

    override fun getContexts(project: Project, file: VirtualFile) = getRoots(project)

//    override fun getRoots(module: Module) = getRoots(module.project)
//
    private fun getRoots(project: Project): Collection<PsiFileSystemItem> {
        val roots = ProjectRootManager.getInstance(project).contentRoots
        val psiManager = PsiManager.getInstance(project)
        return roots.mapNotNull { psiManager.findDirectory(it) }.toList()
    }

    private fun isPackageXml(file: PsiFile): Boolean {
        return if (file !is XmlFile) false else getPackageXml(file) != null
    }

    private fun getPackageXml(file: XmlFile): DomFileElement<PackageDom>? {
        return DomManager.getDomManager(file.project).getFileElement(file, PackageDom::class.java)
    }
}