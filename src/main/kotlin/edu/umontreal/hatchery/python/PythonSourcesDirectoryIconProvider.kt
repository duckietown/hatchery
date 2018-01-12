package edu.umontreal.hatchery.python

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.filesystem.Icons

class PythonSourcesDirectoryIconProvider : IconProvider() {
    override fun getIcon(element: PsiElement, flags: Int) =
            if (element is PsiDirectory && hasPythonSources(element)) Icons.python_dir else null

    private fun hasPythonSources(element: PsiDirectory) = element.name =="src" && element.files.any { it.name.endsWith("py") }
}