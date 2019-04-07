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
      Info(AllIcons.RunConfigurations.TestState.Run, ExecutorAction.getActions()) { "Run ROSLaunch" }
    else null

  private fun isIdentifier(psiEl: PsiElement?): Boolean {
    val isLaunchFile = psiEl?.containingFile?.name?.endsWith(".launch") ?: false
    val isTopmostTag = (psiEl as? XmlTag)?.name == "launch"
    return isLaunchFile && isTopmostTag
  }
}