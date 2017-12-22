package edu.umontreal.hatchery

import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*
import com.intellij.util.ProcessingContext

class PackageReferenceContributor : PsiReferenceContributor() {
    companion object {
        val EXTENSION_TAG_NAMES = arrayOf("build_depend", "run_depend", "test_depend")
    }

    override fun registerReferenceProviders(registrar: PsiReferenceRegistrar) {
        val bundlePattern = createPattern(EXTENSION_TAG_NAMES)
        registrar.registerReferenceProvider(bundlePattern, ROSReferenceProvider)

//        val typeNameBundlePattern = createPattern(TYPE_NAME_TAG, "resourceBundle")
//        registrar.registerReferenceProvider(typeNameBundlePattern, ROSReferenceProvider)

//        val intentionActionBundleTagPattern = XmlPatterns.xmlTag().withName(INTENTION_ACTION_BUNDLE_TAG).withParent(XmlPatterns.xmlTag().withName(INTENTION_ACTION_TAG).withSuperParent(2, XmlPatterns.xmlTag().withName("idea-plugin")))
//        registrar.registerReferenceProvider(intentionActionBundleTagPattern,ROSReferenceProvider)
    }

    private fun createPattern(tagNames: Array<String>, vararg attributeNames: String) =
            XmlPatterns.xmlText().withParent(XmlPatterns.xmlTag().withName(*tagNames))
//                    .withSuperParent(2, XmlPatterns.xmlTag().withName("package"))


    private object ROSReferenceProvider : PsiReferenceProvider() {
        override fun getReferencesByElement(el: PsiElement, ctx: ProcessingContext) = arrayOf(MyBundleReference(el))
    }

    private class MyBundleReference(element: PsiElement) : PsiReferenceBase<PsiElement>(element, false) {
        override fun resolve() = element

        override fun getVariants() = arrayOf("test1", "test2", "test3")
    }
}