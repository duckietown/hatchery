package edu.umontreal.hatchery.rospackage

import com.intellij.psi.PsiReferenceContributor
import com.intellij.psi.PsiReferenceRegistrar
import edu.umontreal.hatchery.rospackage.RosPackagePsiReferenceProvider.XML_PATTERN

object RosPackageReferenceContributor : PsiReferenceContributor() {
  override fun registerReferenceProviders(psiReferenceRegistrar: PsiReferenceRegistrar) =
    psiReferenceRegistrar.registerReferenceProvider(XML_PATTERN, RosPackagePsiReferenceProvider)
}