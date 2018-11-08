package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.execution.lineMarker.ExecutorAction
import com.intellij.execution.lineMarker.RunLineMarkerContributor
import com.intellij.icons.AllIcons
import com.intellij.psi.PsiElement
import com.intellij.psi.xml.XmlTag

/**
 * @see com.intellij.execution.application.ApplicationRunLineMarkerProvider
 */

object RosLaunchRunLineMarkerContributor : RunLineMarkerContributor() {
  override fun getInfo(element: PsiElement) =
    if (isIdentifier(element))
      Info(AllIcons.General.Run, ExecutorAction.getActions()) { "Run ROSLaunch Configuration" }
    else null

  private fun isIdentifier(element: PsiElement?): Boolean {
    val isLaunchFile = element?.containingFile?.name?.endsWith(".launch") ?: false
    val isTopmostTag = (element as? XmlTag)?.name == "launch"
    return isLaunchFile && isTopmostTag
  }
}