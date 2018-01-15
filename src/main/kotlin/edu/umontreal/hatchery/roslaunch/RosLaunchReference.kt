package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue

class RosLaunchReference(element: PsiElement) : PsiReferenceBase<PsiElement>(element, false) {
  override fun resolve(): PsiElement? {
    val relativePath = (element as XmlAttributeValue).value!!.substringAfter("$(find ").replace(")", "")
    return RosLaunchLineMarkerProvider.findFilesByRelativePath(element.project, relativePath).firstOrNull()
  }

  override fun getVariants() = arrayOf<String>()
}