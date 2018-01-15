package edu.umontreal.hatchery.rospackage

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlTag
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.rospackage.RosPackagePsiReferenceProvider.DEPEND_TAG_NAMES

object RosPackageLineMarkerProvider : RelatedItemLineMarkerProvider() {
  private const val TOOLTIP_TEXT = "ROS Package Dependency"

  override fun collectNavigationMarkers(element: PsiElement, results: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
    if (element !is XmlTag || element.name in DEPEND_TAG_NAMES) return

    val scope = GlobalSearchScope.allScope(element.project)
    val files = FilenameIndex.getFilesByName(element.project, RosPackageFileType.filename, scope)
    val directories = files.filter { it.containingDirectory.name == element.value.text }.map { it.containingDirectory }

    if (directories.isEmpty()) return

    val lineMarkerInfo = NavigationGutterIconBuilder
        .create(Icons.package_file)
        .setTargets(directories)
        .setTooltipText(TOOLTIP_TEXT)
        .createLineMarkerInfo(element)

    results.add(lineMarkerInfo)
  }
}