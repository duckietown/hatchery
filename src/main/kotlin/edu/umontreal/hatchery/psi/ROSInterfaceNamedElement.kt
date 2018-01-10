package edu.umontreal.hatchery.psi

import com.intellij.psi.PsiNameIdentifierOwner

interface ROSInterfaceNamedElement : PsiNameIdentifierOwner {
    fun getKey(): String?

    fun getValue(): String?

    fun getType(): String?
}