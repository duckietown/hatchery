package edu.umontreal.hatchery.rospackage

import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.search.FileTypeIndex
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlFile
import com.intellij.psi.xml.XmlTag
import com.jetbrains.extensions.python.toPsi
import edu.umontreal.hatchery.rospackage.RosPackageReferenceContributor.rosPackages

class RosPackageReference(private val psiElement: XmlTag) : PsiReferenceBase<XmlTag>(psiElement, false) {
  override fun resolve() =
    psiElement.let { tag ->
      val packageName = tag.value.text
      getLocalRosPackages().firstOrNull { it.containingDirectory.name == packageName }
        ?: if (rosPackages.containsKey(packageName)) psiElement else null
    }

  override fun getVariants() =
    getLocalRosPackages().flatMap { file ->
      file.document?.rootTag?.subTags
        ?.filter { it.name == "name" }?.map { it.value.text } ?: listOf()
    }.toTypedArray()


//  private fun getLocalRosPackages() =
//    FileTypeIndex.getFiles(RosPackageFileType, GlobalSearchScope.allScope(psiElement.project))
//      .map { it.toPsi(psiElement.project) }.filterIsInstance<XmlFile>()
  private fun getLocalRosPackages() =
    FilenameIndex.getFilesByName(psiElement.project,
      RosPackageFileType.filename,
      GlobalSearchScope.allScope(psiElement.project)
    ).filterIsInstance<XmlFile>()
}