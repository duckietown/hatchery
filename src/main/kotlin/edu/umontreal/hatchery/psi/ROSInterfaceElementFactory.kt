package edu.umontreal.hatchery.psi

import com.intellij.openapi.project.Project
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFileFactory
import edu.umontreal.hatchery.rosinterface.ROSInterfaceFileFactory

object ROSInterfaceElementFactory {
    fun createProperty(project: Project, name: String, value: String): ROSInterfaceProperty {
        val file = createFile(project, name + " = " + value)
        return file.firstChild as ROSInterfaceProperty
    }

    fun createProperty(project: Project, name: String): ROSInterfaceProperty {
        val file = createFile(project, name)
        return file.firstChild as ROSInterfaceProperty
    }

    fun createCRLF(project: Project): PsiElement {
        val file = createFile(project, "\n")
        return file.firstChild
    }

    fun createFile(project: Project, text: String): ROSInterfaceFile {
        val name = "dummy.ROSInterface"
        return PsiFileFactory.getInstance(project).createFileFromText(name, ROSInterfaceFileFactory.ROSInterfaceFileType, text) as ROSInterfaceFile
    }
}