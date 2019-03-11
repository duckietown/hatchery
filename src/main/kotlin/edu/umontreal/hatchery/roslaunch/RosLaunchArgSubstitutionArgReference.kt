package edu.umontreal.hatchery.roslaunch

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue
import com.intellij.psi.xml.XmlFile

class RosLaunchArgSubstitutionArgReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  override fun resolve(): PsiElement? {
    val attributeVal = element.value
    val argName = attributeVal.substringAfter("$(arg ").substringBefore(")")
    val file = element.containingFile as? XmlFile
    val argTags = file?.rootTag?.findSubTags("arg")
    val argTagMatch = argTags?.first { it.getAttribute("name")?.value == argName }

    return argTagMatch
  }

  override fun getVariants() = arrayOf<String>()
}