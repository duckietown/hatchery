package edu.umontreal.hatchery.catkin

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.filesystem.Icons

object CatkinIconProvider : IconProvider() {
  override fun getIcon(element: PsiElement, flags: Int) =
    if (element is PsiDirectory && element.hasPackageXml()) Icons.catkin_file else null

  private fun PsiDirectory.hasPackageXml() = files.any { it.name == "package.xml" }
}