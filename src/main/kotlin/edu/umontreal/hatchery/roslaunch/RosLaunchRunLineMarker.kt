package edu.umontreal.hatchery.roslaunch

import com.intellij.execution.lineMarker.RunLineMarkerContributor
import com.intellij.icons.AllIcons
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.XmlTag

class RosLaunchRunLineMarker : RunLineMarkerContributor() {
  override fun getInfo(element: PsiElement): Info? {
    val filename = element.containingFile.name
    if (filename.endsWith(".launch") && element is XmlTag && element.name == "launch")
      return Info(AllIcons.General.Run, { "" }, arrayOf(
          RosLaunchRunTargetAction(filename + " (robot)"),
          RosLaunchRunTargetAction(filename + " (local)")))

    return null
  }
}