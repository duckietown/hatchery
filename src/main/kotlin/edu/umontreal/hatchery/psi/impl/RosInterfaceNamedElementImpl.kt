package edu.umontreal.hatchery.psi.impl

import com.intellij.extapi.psi.ASTWrapperPsiElement
import com.intellij.lang.ASTNode
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.psi.*

open class RosInterfaceNamedElementImpl(node: ASTNode) : ASTWrapperPsiElement(node), RosInterfaceNamedElement {
  override fun getType() = node.findChildByType(RosInterfaceTypes.TYPE)?.text

  override fun getKey(): String? {
    val keyNode = node.findChildByType(RosInterfaceTypes.KEY)
    return keyNode?.text?.replace("\\\\ ".toRegex(), " ")
  }

  override fun getValue() = node.findChildByType(RosInterfaceTypes.VALUE)?.text

  override fun getName() = getKey()

  override fun setName(newName: String): PsiElement {
    val keyNode = node.findChildByType(RosInterfaceTypes.KEY)
    if (keyNode != null) {
      val property = RosInterfaceElementFactory.createProperty(project, newName)
      val newKeyNode = property.firstChild.node
      node.replaceChild(keyNode, newKeyNode)
    }

    return this
  }

  override fun getNameIdentifier() = node.findChildByType(RosInterfaceTypes.KEY)?.psi
}