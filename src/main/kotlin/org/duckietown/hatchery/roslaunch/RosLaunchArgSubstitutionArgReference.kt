package org.duckietown.hatchery.roslaunch

import com.intellij.psi.*
import com.intellij.psi.xml.*

class RosLaunchArgSubstitutionArgReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  override fun resolve(): PsiElement? {
    val attributeVal = element.value
    val argName = attributeVal.substringAfter("$(arg ").substringBefore(")")
    val file = element.containingFile as? XmlFile
    val argTags = file?.rootTag?.findSubTags("arg")

    return argTags?.firstOrNull { it.getAttribute("name")?.value == argName }
  }

  override fun getVariants() = arrayOf<String>()
}