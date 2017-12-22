package edu.umontreal.hatchery

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.XmlText

class ROSPackageLineMarkerProvider : RelatedItemLineMarkerProvider() {
    override fun collectNavigationMarkers(element: PsiElement, result: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
        if (element is XmlText && element.parentTag?.name in PackageReferenceContributor.EXTENSION_TAG_NAMES) {
            val builder = NavigationGutterIconBuilder.create(Icons.package_file)
                    .setTarget(element).setTooltipText("ROS Package Dependency")
            result.add(builder.createLineMarkerInfo(element))
        }
    }
}