package org.duckietown.hatchery.roslaunch

import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.xml.XmlAttributeValue

class RosLaunchSourceFileReference : PsiReferenceBase<XmlAttributeValue> {
  constructor(element: XmlAttributeValue) : super(element, false)

  override fun resolve() = element.containingFile.containingDirectory
    .parentDirectory?.findSubdirectory("src")?.findFile(element.value)

  override fun getVariants() = arrayOf<String>()
}