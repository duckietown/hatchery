package edu.umontreal.hatchery.rospackage

import com.intellij.psi.PsiElement
import com.intellij.psi.PsiReferenceBase
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlFile
import com.intellij.psi.xml.XmlTag

class RosPackageReference(private val psiElement: PsiElement) : PsiReferenceBase<PsiElement>(psiElement, false) {
  override fun resolve() =
      (psiElement as? XmlTag)?.let { tag ->
        getAllROSPackages().firstOrNull { it.containingDirectory.name == tag.value.text }?.containingDirectory
      }

  override fun getVariants() =
      getAllROSPackages().flatMap { file ->
        file.document?.rootTag?.subTags?.filter { it.name == "name" }?.map { it.value.text } ?: listOf()
      }.toTypedArray()

  private fun getAllROSPackages() =
      FilenameIndex.getFilesByName(psiElement.project, "package.xml", GlobalSearchScope.allScope(psiElement.project))
          .filterIsInstance<XmlFile>()
}