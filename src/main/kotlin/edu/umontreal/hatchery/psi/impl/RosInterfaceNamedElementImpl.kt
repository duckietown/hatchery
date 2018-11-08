package edu.umontreal.hatchery.psi.impl

import com.intellij.extapi.psi.ASTWrapperPsiElement
import com.intellij.lang.ASTNode
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.psi.RosInterfaceElementFactory
import edu.umontreal.hatchery.psi.RosInterfaceNamedElement
import edu.umontreal.hatchery.psi.RosInterfaceTypes.*

open class RosInterfaceNamedElementImpl(node: ASTNode) : ASTWrapperPsiElement(node), RosInterfaceNamedElement {
  override fun getType() = node.findChildByType(TYPE)?.text

  override fun getKey() = node.findChildByType(KEY)?.text?.replace("\\\\ ".toRegex(), " ")

  override fun getValue() = node.findChildByType(VALUE)?.text

  override fun getName() = getKey()

  override fun setName(newName: String): PsiElement {
    val keyNode = node.findChildByType(KEY)
    if (keyNode != null) {
      val property = RosInterfaceElementFactory.createProperty(project, newName)
      val newKeyNode = property.firstChild.node
      node.replaceChild(keyNode, newKeyNode)
    }

    return this
  }

  override fun getNameIdentifier() = node.findChildByType(KEY)?.psi
}