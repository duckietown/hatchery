package edu.umontreal.hatchery.rospackage

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.PsiReferenceContributor
import com.intellij.psi.PsiReferenceRegistrar
import edu.umontreal.hatchery.ros.Ros
import edu.umontreal.hatchery.settings.RosConfig

object RosPackageReferenceContributor : PsiReferenceContributor() {
  val DEPEND_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
  private val XML_PATTERN = XmlPatterns.xmlTag().withLocalName(*DEPEND_TAG_NAMES)!!
  val rosPackages: Map<String, String> by lazy {
    Ros(RosConfig.settings.localRosPath).pack.list.call().map {
      if(it.contains(" "))
        Pair(it.substringBefore(" "), it.substringAfter(" ") + "/package.xml")
      else
        Pair(it, "")
    }.toMap()
  }

  override fun registerReferenceProviders(psiReferenceRegistrar: PsiReferenceRegistrar) =
    psiReferenceRegistrar.registerReferenceProvider(XML_PATTERN, RosPackagePsiReferenceProvider)
}