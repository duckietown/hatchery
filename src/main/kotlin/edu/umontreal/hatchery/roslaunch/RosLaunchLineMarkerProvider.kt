package edu.umontreal.hatchery.roslaunch

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.XmlAttributeValue
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.util.findFilesByRelativePath


object RosLaunchLineMarkerProvider : RelatedItemLineMarkerProvider() {
  override fun collectNavigationMarkers(element: PsiElement, result: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
    if (!isRosLaunchFileSubstitution(element)) return

    val relPath = (element as XmlAttributeValue).value.substringAfter("$(find ").replace(")", "")
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
}