package org.duckietown.hatchery.rosmanifest

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*

object RosManifestReferenceContributor : PsiReferenceContributor() {
  val DEPEND_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
  private val XML_PATTERN = XmlPatterns.xmlTag().withLocalName(*DEPEND_TAG_NAMES)!!

  override fun registerReferenceProviders(psiReferenceRegistrar: PsiReferenceRegistrar) =
    psiReferenceRegistrar.registerReferenceProvider(XML_PATTERN, RosManifestPsiReferenceProvider)
}