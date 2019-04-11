package edu.umontreal.hatchery.python

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.filesystem.Icons
import org.apache.xmlbeans.xml.stream.events.ElementTypeNames

class PythonSourcesDirectoryIconProvider : IconProvider() {
  override fun getIcon(e: PsiElement, f: Int) =
    if (isSourceFolder(e) && (hasPythonFiles(e) || hasPackageSibling(e))) Icons.python_dir else null

  private fun isSourceFolder(element: PsiElement) = element is PsiDirectory && element.name == "src"

  private fun hasPythonFiles(element: PsiElement) = (element as PsiDirectory).files.any { it.name.endsWith(".py") }

  private fun hasPackageSibling(element: PsiElement) =
    (element as PsiDirectory).parentDirectory?.files?.any { it.name == "package.xml" } ?: false
}