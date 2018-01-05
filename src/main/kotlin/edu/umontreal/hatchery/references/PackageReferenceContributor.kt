package edu.umontreal.hatchery.references

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*
import com.intellij.util.ProcessingContext

class PackageReferenceContributor : PsiReferenceContributor() {
    companion object {
        val EXTENSION_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
    }

    override fun registerReferenceProviders(registrar: PsiReferenceRegistrar) =
            registrar.registerReferenceProvider(createPattern(EXTENSION_TAG_NAMES), ROSReferenceProvider)

    private fun createPattern(tagNames: Array<String>) =
            XmlPatterns.xmlText().withParent(XmlPatterns.xmlTag().withName(*tagNames))


    private object ROSReferenceProvider : PsiReferenceProvider() {
        override fun getReferencesByElement(el: PsiElement, ctx: ProcessingContext) = arrayOf(MyBundleReference(el))
    }

    private class MyBundleReference(element: PsiElement) : PsiReferenceBase<PsiElement>(element, false) {
        override fun resolve() = element

        override fun getVariants() = arrayOf("test1", "test2", "test3")
    }
}