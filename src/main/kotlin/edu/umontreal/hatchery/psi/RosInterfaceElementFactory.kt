package edu.umontreal.hatchery.psi

import com.intellij.openapi.project.Project
import com.intellij.psi.PsiFileFactory
import edu.umontreal.hatchery.rosinterface.RosInterfaceFileType

object RosInterfaceElementFactory {
  fun createProperty(project: Project, name: String, value: String): RosInterfaceProperty {
    val file = createFile(project, "$name = $value")
    return file.firstChild as RosInterfaceProperty
  }

  fun createProperty(project: Project, name: String): RosInterfaceProperty {
    val file = createFile(project, name)
    return file.firstChild as RosInterfaceProperty
  }

  fun createCRLF(project: Project) = createFile(project, "\n").firstChild

  fun createFile(project: Project, text: String): RosInterfaceFile {
    val name = "dummy.RosInterface"
    return PsiFileFactory.getInstance(project).createFileFromText(name, RosInterfaceFileType, text) as RosInterfaceFile
  }
}