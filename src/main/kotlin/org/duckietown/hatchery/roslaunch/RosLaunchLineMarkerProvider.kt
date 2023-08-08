package org.duckietown.hatchery.roslaunch

import com.intellij.codeInsight.daemon.*
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.*
import org.duckietown.hatchery.filesystem.Icons
import org.duckietown.hatchery.util.findFilesByRelativePath


class RosLaunchLineMarkerProvider : RelatedItemLineMarkerProvider() {
  override fun collectNavigationMarkers(element: PsiElement, result: MutableCollection<in RelatedItemLineMarkerInfo<*>>) {
    if (isRosLaunchFileSubstitution(element)) {
      val relPath = (element as XmlAttributeValue).value.substringAfter("$(find ").replace(")", "")
      if (!relPath.contains("(")) {
        val builder = NavigationGutterIconBuilder.create(Icons.resource_file)
        val targets = findFilesByRelativePath(element.project, relPath)

        targets.firstOrNull()?.virtualFile?.path?.let {
          builder.setTooltipText(it).setTargets(targets)
          result.add(builder.createLineMarkerInfo(element))
        }
          ?: result.add(NavigationGutterIconBuilder.create(Icons.broken_resource)
            .setTooltipText("Unknown resource!")
            .setTarget(element).createLineMarkerInfo(element))
      }
    }
//    else if(isRosLaunchArg(element)) {
//      val builder = NavigationGutterIconBuilder.create(Icons.resource_file)
//
//      targets.firstOrNull()?.virtualFile?.path?.let {
//        result.add(builder.createLineMarkerInfo(element))
//      }
//    }
  }

  private fun isRosLaunchArg(element: PsiElement): Boolean =
    (element as? XmlTag)?.run { name == "arg" && getAttribute("name") != null } ?: false

  private fun isRosLaunchFileSubstitution(element: PsiElement): Boolean =
    (element as? XmlAttributeValue)?.value?.startsWith("$(find ") ?: false

  private fun isRosLaunchSourceFile(element: PsiElement): Boolean =
    (element as? XmlAttributeValue)?.value?.matches(Regex("[A-Za-z0-9_]*\\.py")) ?: false
}