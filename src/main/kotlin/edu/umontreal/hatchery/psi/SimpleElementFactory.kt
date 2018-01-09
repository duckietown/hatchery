package edu.umontreal.hatchery.psi

import com.intellij.openapi.project.Project
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFileFactory
import edu.umontreal.hatchery.rosmsg.ROSInterfaceFileFactory

object SimpleElementFactory {
    fun createProperty(project: Project, name: String, value: String): SimpleProperty {
        val file = createFile(project, name + " = " + value)
        return file.firstChild as SimpleProperty
    }

    fun createProperty(project: Project, name: String): SimpleProperty {
        val file = createFile(project, name)
        return file.firstChild as SimpleProperty
    }

    fun createCRLF(project: Project): PsiElement {
        val file = createFile(project, "\n")
        return file.firstChild
    }

    fun createFile(project: Project, text: String): SimpleFile {
        val name = "dummy.simple"
        return PsiFileFactory.getInstance(project).createFileFromText(name, ROSInterfaceFileFactory.ROSInterfaceFileType, text) as SimpleFile
    }
}