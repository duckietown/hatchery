package edu.umontreal.hatchery.psi.impl

import com.intellij.extapi.psi.ASTWrapperPsiElement
import com.intellij.lang.ASTNode
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.psi.RosInterfaceElementFactory
import edu.umontreal.hatchery.psi.RosInterfaceNamedElement
import edu.umontreal.hatchery.psi.ROSInterfaceTypes

open class RosInterfaceNamedElementImpl(node: ASTNode) : ASTWrapperPsiElement(node), RosInterfaceNamedElement {
  override fun getType() = node.findChildByType(ROSInterfaceTypes.TYPE)?.text

  override fun getKey(): String? {
    val keyNode = node.findChildByType(ROSInterfaceTypes.KEY)
    return keyNode?.text?.replace("\\\\ ".toRegex(), " ")
  }

  override fun getValue() = node.findChildByType(ROSInterfaceTypes.VALUE)?.text

  override fun getName() = getKey()

  override fun setName(newName: String): PsiElement {
    val keyNode = node.findChildByType(ROSInterfaceTypes.KEY)
    if (keyNode != null) {
      val property = RosInterfaceElementFactory.createProperty(project, newName)
      val newKeyNode = property.firstChild.node
      node.replaceChild(keyNode, newKeyNode)
    }

    return this
  }

  override fun getNameIdentifier() = node.findChildByType(ROSInterfaceTypes.KEY)?.psi
}