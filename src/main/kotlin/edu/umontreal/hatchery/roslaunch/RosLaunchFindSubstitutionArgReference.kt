package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue
import edu.umontreal.hatchery.util.findFilesByRelativePath

class RosLaunchFindSubstitutionArgReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  // TODO: Make this smarter
  override fun resolve(): PsiElement? {
    val attributeVal = element.value
    val relativePath = attributeVal.substringAfter("$(find ").replace(")", "")
    return findFilesByRelativePath(element.project, relativePath).firstOrNull()
  }

  override fun getVariants() = arrayOf<String>()
}