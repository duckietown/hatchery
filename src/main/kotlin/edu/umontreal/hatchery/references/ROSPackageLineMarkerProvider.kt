package edu.umontreal.hatchery.references

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.XmlText
import edu.umontreal.hatchery.references.PackageReferenceContributor.Companion.EXTENSION_TAG_NAMES
import edu.umontreal.hatchery.filesystem.Icons

class ROSPackageLineMarkerProvider : RelatedItemLineMarkerProvider() {
    override fun collectNavigationMarkers(element: PsiElement, results: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
        if (element is XmlText && element.parentTag?.name in EXTENSION_TAG_NAMES) {
            val builder = NavigationGutterIconBuilder.create(Icons.package_file).setTarget(element).setTooltipText("ROS Package Dependency")
            results.add(builder.createLineMarkerInfo(element))
        }

    }
}