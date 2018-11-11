package edu.umontreal.hatchery.roslaunch

import com.intellij.patterns.StandardPatterns
import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*
import com.intellij.util.ProcessingContext

object RosLaunchReferenceContributor : PsiReferenceContributor() {
  // http://wiki.ros.org/roslaunch/XML#substitution_args
  private const val findSubstitution = "\\\$\\(find [\\w]*\\)[\\w/\\.]*"
  private val pattern = XmlPatterns.xmlAttributeValue()
    .withValue(StandardPatterns.string().matches(findSubstitution))!!

  override fun registerReferenceProviders(psiReg: PsiReferenceRegistrar) =
    psiReg.registerReferenceProvider(pattern, object : PsiReferenceProvider() {
      override fun getReferencesByElement(e: PsiElement, c: ProcessingContext) =
        arrayOf(RosLaunchReference(e))
    })
}