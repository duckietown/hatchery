package edu.umontreal.hatchery.roslaunch

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.openapi.fileTypes.FileTypeManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFileSystemItem
import com.intellij.psi.PsiManager
import com.intellij.psi.search.FileTypeIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlAttributeValue
import edu.umontreal.hatchery.filesystem.Icons


object RosLaunchLineMarkerProvider : RelatedItemLineMarkerProvider() {
  override fun collectNavigationMarkers(element: PsiElement, result: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
    if (!isRosLaunchFileSubstitution(element)) return

    val relPath = (element as XmlAttributeValue).value!!.substringAfter("$(find ").replace(")", "")
    if (!relPath.contains("(")) {
      val builder = NavigationGutterIconBuilder.create(Icons.resource_file)
      val targets = findFilesByRelativePath(element.project, relPath)

      targets.firstOrNull()?.virtualFile?.path?.let {
        builder.setTooltipText(it).setTargets(targets)
        result.add(builder.createLineMarkerInfo(element))
      } ?: result.add(NavigationGutterIconBuilder.create(Icons.broken_resource)
        .setTooltipText("Unknown resource!")
        .setTarget(element).createLineMarkerInfo(element))
    }
  }

  private fun isRosLaunchFileSubstitution(element: PsiElement): Boolean =
    (element as? XmlAttributeValue)?.value?.startsWith("$(find ") ?: false

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
}