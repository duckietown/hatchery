package edu.umontreal.hatchery.rospackage

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceProvider
import com.intellij.psi.xml.XmlTag
import com.intellij.util.ProcessingContext

object RosPackagePsiReferenceProvider : PsiReferenceProvider() {
  override fun getReferencesByElement(e: PsiElement, c: ProcessingContext) =
    (e as? XmlTag)?.let { arrayOf(RosPackageReference(e)) } ?: emptyArray()
}