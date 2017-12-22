package edu.umontreal.hatchery


import com.intellij.ExtensionPoints
import com.intellij.ide.presentation.Presentation
import com.intellij.util.xml.*

/**
 * plugin.dtd:idea-plugin interface.
 */
@DefinesXml
@Presentation(icon = "AllIcons.Nodes.Plugin", typeName = "Plugin")
@Stubbed
interface PackageDom : DomElement {
    val id: GenericDomValue<String>
    val name: GenericDomValue<String>
    val description: GenericDomValue<String>
    val version: GenericDomValue<String>
    val author: GenericDomValue<String>
    val maintainer: GenericDomValue<String>
    val license: GenericDomValue<String>
    val buildtool_depend: GenericDomValue<String>
    val build_depend: GenericDomValue<String>
    val run_depend: GenericDomValue<String>
    val export: GenericDomValue<String>
}