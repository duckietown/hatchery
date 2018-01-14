package edu.umontreal.hatchery.psi

import com.intellij.psi.PsiNameIdentifierOwner

interface RosInterfaceNamedElement : PsiNameIdentifierOwner {
  fun getKey(): String?

  fun getValue(): String?

  fun getType(): String?
}