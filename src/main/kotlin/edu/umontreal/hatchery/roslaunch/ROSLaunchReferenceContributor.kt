package edu.umontreal.hatchery.roslaunch

import com.intellij.patterns.StandardPatterns
import com.intellij.patterns.XmlPatterns
import com.intellij.psi.*
import com.intellij.psi.xml.XmlAttributeValue
import com.intellij.util.ProcessingContext

class ROSLaunchReferenceContributor : PsiReferenceContributor() {
    override fun registerReferenceProviders(registrar: PsiReferenceRegistrar) =
            registrar.registerReferenceProvider(createPattern(), ReferenceProvider)

    private fun createPattern() =
            XmlPatterns.xmlAttributeValue().withValue(StandardPatterns.string().matches("\\\$\\(find [\\w]*\\)[\\w/\\.]*"))

    private object ReferenceProvider : PsiReferenceProvider() {
        override fun getReferencesByElement(el: PsiElement, ctx: ProcessingContext) = arrayOf(MyBundleReference(el))
    }

    private class MyBundleReference(element: PsiElement) : PsiReferenceBase<PsiElement>(element, false) {
        override fun resolve(): PsiElement? {
            val manager = PsiManager.getInstance(element.project)
            val relPath = (element as XmlAttributeValue).value!!.substringAfter("$(find ").replace(")", "")
            val file = ROSLaunchLineMarkerProvider.findFileByRelativePath(element.project, relPath).firstOrNull()
            return file?.let { manager.findFile(file) ?: manager.findDirectory(file) }
        }

        override fun getVariants() = arrayOf<String>()
    }
}