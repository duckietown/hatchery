package edu.umontreal.hatchery.rospackage

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlFile
import com.intellij.psi.xml.XmlTag
import com.intellij.util.ProcessingContext

class ROSPackageReferenceContributor : PsiReferenceContributor() {
    companion object {
        val EXTENSION_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
    }

    override fun registerReferenceProviders(registrar: PsiReferenceRegistrar) =
            registrar.registerReferenceProvider(createPattern(EXTENSION_TAG_NAMES), ROSReferenceProvider)

    private fun createPattern(tagNames: Array<String>) = XmlPatterns.xmlTag().withLocalName(*tagNames)

    private object ROSReferenceProvider : PsiReferenceProvider() {
        override fun getReferencesByElement(element: PsiElement, ctx: ProcessingContext) = arrayOf(MyBundleReference(element))
    }

    private class MyBundleReference(element: PsiElement) : PsiReferenceBase<PsiElement>(element, false) {
        override fun resolve(): PsiElement? {
            val files = FilenameIndex.getFilesByName(element.project, "package.xml", GlobalSearchScope.allScope(element.project))
            return files.firstOrNull { it.containingDirectory.name == (element as XmlTag).value.text }?.containingDirectory
                    ?: element
        }

        override fun getVariants(): Array<String> {
            val files = FilenameIndex.getFilesByName(element.project, "package.xml", GlobalSearchScope.allScope(element.project))

            val availablePackages = mutableListOf<String>()
            files.map { file ->
                if (file is XmlFile)
                    file.document?.rootTag?.subTags?.forEach {
                        if (it is XmlTag && it.localName == "name") availablePackages.add(it.value.text)
                    }
            }

            return availablePackages.toTypedArray()
        }
    }
}