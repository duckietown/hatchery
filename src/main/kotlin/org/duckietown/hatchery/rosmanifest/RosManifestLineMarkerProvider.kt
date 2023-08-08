package org.duckietown.hatchery.rosmanifest

import com.intellij.codeInsight.daemon.*
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.search.*
import com.intellij.psi.xml.XmlTag
import org.duckietown.hatchery.filesystem.Icons

class RosManifestLineMarkerProvider : RelatedItemLineMarkerProvider() {
  private val TOOLTIP_TEXT = "ROS Package Dependency"

  override fun collectNavigationMarkers(element: PsiElement, results: MutableCollection<in RelatedItemLineMarkerInfo<*>>) {
    if (element !is XmlTag || element.name !in RosManifestReferenceContributor.DEPEND_TAG_NAMES) return

    val scope = GlobalSearchScope.allScope(element.project)
    val files = FilenameIndex.getFilesByName(element.project, RosManifestFileType.filename, scope).map { it.containingDirectory }
    val directories = files.filter { it.name == element.value.text }

    if (directories.isEmpty()) return

    val lineMarkerInfo = NavigationGutterIconBuilder
      .create(Icons.package_file)
      .setTargets(directories)
      .setTooltipText(TOOLTIP_TEXT)
      .createLineMarkerInfo(element)

    results.add(lineMarkerInfo)
  }
}