package edu.umontreal.hatchery.rospackage

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.PsiReferenceContributor
import com.intellij.psi.PsiReferenceRegistrar
import edu.umontreal.hatchery.ros.Ros
import edu.umontreal.hatchery.settings.RosConfig
import java.io.File

object RosPackageReferenceContributor : PsiReferenceContributor() {
  val DEPEND_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
  private val XML_PATTERN = XmlPatterns.xmlTag().withLocalName(*DEPEND_TAG_NAMES)!!
  val rosPackages: Map<String, File> by lazy {
    Ros(RosConfig.settings.localRosPath).pack.list.call().lines().dropLast(1).map {
      Pair(it.substringBefore(" "), File(it.substringAfter(" ") + "/package.xml"))
    }.toMap()
  }

  override fun registerReferenceProviders(psiReferenceRegistrar: PsiReferenceRegistrar) =
    psiReferenceRegistrar.registerReferenceProvider(XML_PATTERN, RosPackagePsiReferenceProvider)
}