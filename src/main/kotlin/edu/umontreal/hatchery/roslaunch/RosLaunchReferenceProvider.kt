package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceProvider
import com.intellij.util.ProcessingContext

object RosLaunchReferenceProvider : PsiReferenceProvider() {
  override fun getReferencesByElement(element: PsiElement, ctx: ProcessingContext) = arrayOf(RosLaunchReference(element))
}