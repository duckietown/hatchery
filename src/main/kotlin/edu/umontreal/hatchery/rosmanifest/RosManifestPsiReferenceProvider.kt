package edu.umontreal.hatchery.rosmanifest

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceProvider
import com.intellij.psi.xml.XmlTag
import com.intellij.util.ProcessingContext

object RosManifestPsiReferenceProvider : PsiReferenceProvider() {
  override fun getReferencesByElement(e: PsiElement, c: ProcessingContext) =
    (e as? XmlTag)?.let { arrayOf(RosManifestReference(e)) } ?: emptyArray()
}