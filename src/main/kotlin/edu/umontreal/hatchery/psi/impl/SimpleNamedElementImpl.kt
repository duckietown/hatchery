package edu.umontreal.hatchery.psi.impl

import com.intellij.extapi.psi.ASTWrapperPsiElement
import com.intellij.lang.ASTNode
import edu.umontreal.hatchery.psi.SimpleNamedElement

abstract class SimpleNamedElementImpl(node: ASTNode) : ASTWrapperPsiElement(node), SimpleNamedElement