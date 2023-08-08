package org.duckietown.hatchery.rosmanifest

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*

class RosManifestReferenceContributor : PsiReferenceContributor() {
  companion object {
    val DEPEND_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
    val XML_PATTERN = XmlPatterns.xmlTag().withLocalName(*DEPEND_TAG_NAMES)!!
  }

  override fun registerReferenceProviders(psiReferenceRegistrar: PsiReferenceRegistrar) =
    psiReferenceRegistrar.registerReferenceProvider(XML_PATTERN, RosManifestPsiReferenceProvider)
}