package edu.umontreal.hatchery.completion

import com.intellij.codeInsight.completion.*
import com.intellij.codeInsight.lookup.LookupElementBuilder
import com.intellij.openapi.project.ProjectManager
import com.intellij.patterns.PlatformPatterns.psiElement
import com.intellij.patterns.XmlPatterns
import com.intellij.psi.search.FilenameIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlFile
import com.intellij.psi.xml.XmlTag
import com.intellij.util.ProcessingContext
import com.intellij.util.indexing.ID
import edu.umontreal.hatchery.references.PackageReferenceContributor


class ROSPackageCompletionContributor : CompletionContributor() {
    val PACKAGE_ID = ID.create<String, Void>("package.xml")
    init {
        extend(CompletionType.BASIC, getCapture(), object : CompletionProvider<CompletionParameters>() {
            override fun addCompletions(c: CompletionParameters, p: ProcessingContext?, result: CompletionResultSet) =
                result.addAllElements(getAvailablePackages().map { LookupElementBuilder.create(it) })
        })
    }

    private fun getAvailablePackages(): List<String> {
        val project = ProjectManager.getInstance().openProjects[0]

        val files = FilenameIndex.getFilesByName(project, "package.xml", GlobalSearchScope.allScope(project))
        val availablePackages = mutableListOf<String>()
        files.map { file ->
            if(file is XmlFile) {
                file.document?.rootTag?.subTags?.forEach {
                    if(it is XmlTag && it.localName == "name")
                        availablePackages.add(it.value.text)
                }
            }
        }

        return availablePackages
    }

    private fun getCapture() = psiElement().inside(XmlPatterns.xmlTag().withName(*PackageReferenceContributor.EXTENSION_TAG_NAMES))
}