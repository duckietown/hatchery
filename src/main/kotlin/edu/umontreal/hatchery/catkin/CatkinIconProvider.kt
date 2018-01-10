package edu.umontreal.hatchery.catkin

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFile
import edu.umontreal.hatchery.filesystem.Icons

class CatkinIconProvider : IconProvider() {
    override fun getIcon(element: PsiElement, flags: Int) =
            if (element is PsiDirectory && hasPackageXml(element)) Icons.catkin_file else null

    private fun hasPackageXml(element: PsiDirectory) = element.files.any { it.name == "package.xml" }
}