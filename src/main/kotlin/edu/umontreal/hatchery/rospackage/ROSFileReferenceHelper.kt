package edu.umontreal.hatchery.rospackage

import com.intellij.openapi.project.Project
import com.intellij.openapi.roots.ProjectRootManager
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.PsiFileSystemItem
import com.intellij.psi.PsiManager
import com.intellij.psi.impl.source.resolve.reference.impl.providers.FileReferenceHelper

class ROSFileReferenceHelper : FileReferenceHelper() {
    override fun isMine(project: Project, file: VirtualFile) = file.fileType === ROSPackageFileFactory.ROSPackageFileType

    override fun getContexts(project: Project, file: VirtualFile) = getRoots(project)

    private fun getRoots(project: Project): Collection<PsiFileSystemItem> {
        val roots = ProjectRootManager.getInstance(project).contentRoots
        val psiManager = PsiManager.getInstance(project)
        return roots.mapNotNull { psiManager.findDirectory(it) }.toList()
    }
}