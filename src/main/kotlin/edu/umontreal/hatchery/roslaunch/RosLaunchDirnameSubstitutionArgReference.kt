package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue
import edu.umontreal.hatchery.util.findFilesByRelativePath

class RosLaunchDirnameSubstitutionArgReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  override fun resolve(): PsiElement? {
    val attributeVal = element.value
    val relativePath = attributeVal.substringAfter("$(dirname)")
    return element.containingFile.containingDirectory.findFile(relativePath)
  }

  override fun getVariants() = arrayOf<String>()
}