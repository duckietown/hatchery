package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue

class RosLaunchReference : PsiReferenceBase<PsiElement> {
  constructor(element: PsiElement) : super(element, false)

  // TODO: Make this smarter
  override fun resolve(): PsiElement? {
    val relativePath = (element as XmlAttributeValue).value!!.substringAfter("$(find ").replace(")", "")
    return RosLaunchLineMarkerProvider.findFilesByRelativePath(element.project, relativePath).firstOrNull()
  }

  override fun getVariants() = arrayOf<String>()
}