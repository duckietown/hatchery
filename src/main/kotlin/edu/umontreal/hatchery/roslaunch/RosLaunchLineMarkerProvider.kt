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
import com.intellij.util.indexing.FileBasedIndex
import edu.umontreal.hatchery.filesystem.Icons
import java.util.*


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
      element is XmlAttributeValue && element.value?.startsWith("$(find ") ?: false

  fun findFilesByRelativePath(project: Project, fileRelativePath: String): List<PsiFileSystemItem?> {
    val relativePath = if (fileRelativePath.startsWith("/")) fileRelativePath else "/$fileRelativePath"
    val fileTypes = Collections.singleton(FileTypeManager.getInstance().getFileTypeByFileName(relativePath))
    val fileList = ArrayList<VirtualFile>()
    val manager = PsiManager.getInstance(project)
    val projectScope = GlobalSearchScope.projectScope(project)
    FileBasedIndex.getInstance().processFilesContainingAllKeys(FileTypeIndex.NAME, fileTypes, projectScope, null)
    { virtualFile -> if (virtualFile.path.endsWith(relativePath)) fileList.add(virtualFile); true }
    return fileList.map { manager.run { findFile(it) ?: findDirectory(it) } }
  }
}