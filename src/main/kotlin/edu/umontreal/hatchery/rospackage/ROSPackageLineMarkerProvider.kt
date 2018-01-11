package edu.umontreal.hatchery.rospackage

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlTag
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.rospackage.ROSPackageReferenceContributor.Companion.EXTENSION_TAG_NAMES

class ROSPackageLineMarkerProvider : RelatedItemLineMarkerProvider() {
    override fun collectNavigationMarkers(element: PsiElement, results: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
        if (element is XmlTag && element.name in EXTENSION_TAG_NAMES) {
            val files = FilenameIndex.getFilesByName(element.project, "package.xml", GlobalSearchScope.allScope(element.project))
            val directories = files.filter { it.containingDirectory.name == element.value.text }.map { it.containingDirectory }

            if (directories.isEmpty()) return
            val builder = NavigationGutterIconBuilder.create(Icons.package_file).setTargets(directories).setTooltipText("ROS Package Dependency")
            results.add(builder.createLineMarkerInfo(element))
        }
    }
}