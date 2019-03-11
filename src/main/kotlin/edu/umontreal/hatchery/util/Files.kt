package edu.umontreal.hatchery.util

import com.intellij.openapi.fileTypes.FileTypeManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.PsiFileSystemItem
import com.intellij.psi.PsiManager
import com.intellij.psi.search.FileTypeIndex
import com.intellij.psi.search.GlobalSearchScope
import java.io.File
import java.util.concurrent.TimeUnit
import com.intellij.psi.PsiFile
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.psi.PsiElement
import com.intellij.util.ResourceUtil



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
  return files.mapNotNull { psiMgr.run { findFile(it) ?: findDirectory(it) } }
}

fun String.runCommand() = try {
  val parts = this.split("\\s".toRegex())
  val proc = ProcessBuilder(*parts.toTypedArray())
    .redirectOutput(ProcessBuilder.Redirect.PIPE)
    .redirectError(ProcessBuilder.Redirect.PIPE)
    .start()

  proc.waitFor(60, TimeUnit.SECONDS)
  proc.inputStream.bufferedReader().readText()
} catch (e: Exception) { "" }