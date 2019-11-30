package org.duckietown.hatchery.roslaunch

import com.intellij.psi.*
import com.intellij.psi.xml.XmlAttributeValue

class RosLaunchDirnameSubstitutionArgReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  override fun resolve(): PsiElement? {
    val attributeVal = element.value
    val relativePath = attributeVal.substringAfter("$(dirname)")
    return element.containingFile.containingDirectory.findFile(relativePath)
  }

  override fun getVariants() = arrayOf<String>()
}