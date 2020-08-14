package org.duckietown.hatchery.rosmanifest

import com.intellij.openapi.project.Project
import com.intellij.openapi.roots.ProjectRootManager
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.*
import com.intellij.psi.impl.source.resolve.reference.impl.providers.FileReferenceHelper

object RosManifestFileReferenceHelper : FileReferenceHelper() {
  override fun isMine(project: Project, file: VirtualFile) = file.fileType === RosManifestFileType

  override fun getContexts(project: Project, file: VirtualFile) = getRoots(project)

  private fun getRoots(project: Project): Collection<PsiFileSystemItem> {
    val roots = ProjectRootManager.getInstance(project).contentRoots
    val psiManager = PsiManager.getInstance(project)
    return roots.mapNotNull { psiManager.findDirectory(it) }.toList()
  }
}