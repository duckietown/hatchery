package edu.umontreal.hatchery.rospackage

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceProvider
import com.intellij.util.ProcessingContext

object RosPackagePsiReferenceProvider : PsiReferenceProvider() {
  val DEPEND_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
  val XML_PATTERN = XmlPatterns.xmlTag().withLocalName(*DEPEND_TAG_NAMES)!!

  override fun getReferencesByElement(e: PsiElement, c: ProcessingContext) =
    arrayOf(RosPackageReference(e))
}