package edu.umontreal.hatchery

import com.intellij.codeInsight.completion.*
import com.intellij.codeInsight.lookup.LookupElementBuilder
import com.intellij.patterns.PlatformPatterns.psiElement
import com.intellij.patterns.XmlPatterns
import com.intellij.util.ProcessingContext


class RosPackageCompletionContributor : CompletionContributor() {
    init {
        extend(CompletionType.BASIC, getCapture(), object : CompletionProvider<CompletionParameters>() {
            override fun addCompletions(parameters: CompletionParameters, context: ProcessingContext?, result: CompletionResultSet) {
                result.addElement(LookupElementBuilder.create("Hello"))
            }
        })
    }

    private fun getCapture() = psiElement().inside(XmlPatterns.xmlTag().withName(*PackageReferenceContributor.EXTENSION_TAG_NAMES))
}