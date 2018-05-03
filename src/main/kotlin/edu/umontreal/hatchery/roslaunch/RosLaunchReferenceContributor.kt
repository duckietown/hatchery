package edu.umontreal.hatchery.roslaunch

import com.intellij.patterns.StandardPatterns
import com.intellij.patterns.XmlPatterns
import com.intellij.psi.PsiReferenceContributor
import com.intellij.psi.PsiReferenceRegistrar

object RosLaunchReferenceContributor : PsiReferenceContributor() {
  // http://wiki.ros.org/roslaunch/XML#substitution_args
  const val findSubstitution = "\\\$\\(find [\\w]*\\)[\\w/\\.]*"
  val referencePattern = XmlPatterns.xmlAttributeValue().withValue(StandardPatterns.string().matches(findSubstitution))!!

  override fun registerReferenceProviders(registrar: PsiReferenceRegistrar) =
      registrar.registerReferenceProvider(referencePattern, RosLaunchReferenceProvider)
}