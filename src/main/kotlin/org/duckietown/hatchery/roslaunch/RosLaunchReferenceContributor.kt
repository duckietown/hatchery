package org.duckietown.hatchery.roslaunch

import com.intellij.patterns.*
import com.intellij.psi.*
import com.intellij.psi.xml.XmlAttributeValue
import com.intellij.util.ProcessingContext

object RosLaunchReferenceContributor : PsiReferenceContributor() {
  // http://wiki.ros.org/roslaunch/XML#substitution_args
  private const val findSubstitution = "\\\$\\(find [\\w]*\\)[\\w/\\.]*"
  private const val argSubstitution = "\\\$\\(arg [\\w]*\\)"
  private const val dirnameSubstitution = "\\\$\\(dirname\\)[\\w/\\.]*"
  private const val pythonFileSubstitution = "[A-Za-z0-9_]*\\.py"

  private fun matcher(pattern: String) = XmlPatterns.xmlAttributeValue().withValue(StandardPatterns.string().matches(pattern))!!

  private fun PsiReferenceRegistrar.bind(pattern: String, reference: (XmlAttributeValue) -> PsiReferenceBase<XmlAttributeValue>) =
    registerReferenceProvider(matcher(pattern), object : PsiReferenceProvider() {
      override fun getReferencesByElement(e: PsiElement, c: ProcessingContext) =
        (e as? XmlAttributeValue)?.let { arrayOf(reference(e)) } ?: emptyArray()
    })

  override fun registerReferenceProviders(psiReg: PsiReferenceRegistrar) =
    psiReg.run {
      bind(findSubstitution) { RosLaunchFindSubstitutionArgReference(it) }
      bind(argSubstitution) { RosLaunchArgSubstitutionArgReference(it) }
      bind(dirnameSubstitution) { RosLaunchDirnameSubstitutionArgReference(it) }
      bind(pythonFileSubstitution) { RosLaunchSourceFileReference(it) }
    }
}