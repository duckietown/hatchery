package org.duckietown.hatchery.util

import com.intellij.openapi.fileTypes.FileTypeManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.*
import com.intellij.psi.search.*

fun findFilesByRelativePath(project: Project, fileRelativePath: String): List<PsiFileSystemItem?> {
  val relativePath = if (fileRelativePath.startsWith("/")) fileRelativePath else "/$fileRelativePath"
  val fileType = FileTypeManager.getInstance().getFileTypeByFileName(relativePath)
  val files = mutableListOf<VirtualFile>()
  val psiMgr = PsiManager.getInstance(project)
  val projectScope = GlobalSearchScope.projectScope(project)
  val fileProcessor = { virtualFile: VirtualFile ->
    if (virtualFile.path.endsWith(relativePath)) files.add(virtualFile); true
  }

  FileTypeIndex.processFiles(fileType, fileProcessor, projectScope)
  return files.map { psiMgr.run { findFile(it) ?: findDirectory(it) } }
}