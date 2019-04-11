package edu.umontreal.hatchery.rosmanifest

import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlFile
import com.intellij.psi.xml.XmlTag
import edu.umontreal.hatchery.settings.RosConfig

class RosManifestReference(private val psiElement: XmlTag) : PsiReferenceBase<XmlTag>(psiElement, false) {
  override fun resolve() =
    psiElement.let { tag ->
      val packageName = tag.value.text
      getProjectLocalRosPackages().firstOrNull { it.containingDirectory.name == packageName }
        ?: if (RosConfig.settings.localRos.packages.containsKey(packageName)) psiElement else null
    }

  override fun getVariants() =
    getProjectLocalRosPackages().flatMap { file ->
      file.document?.rootTag?.subTags
        ?.filter { it.name == "name" }?.map { it.value.text } ?: listOf()
    }.toTypedArray()

  private fun getProjectLocalRosPackages() =
    FilenameIndex.getFilesByName(psiElement.project,
      RosManifestFileType.filename,
      GlobalSearchScope.allScope(psiElement.project)
    ).filterIsInstance<XmlFile>()
}