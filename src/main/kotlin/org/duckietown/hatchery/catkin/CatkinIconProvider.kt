package org.duckietown.hatchery.catkin

import com.intellij.ide.IconProvider
import com.intellij.psi.*
import org.duckietown.hatchery.filesystem.Icons

class CatkinIconProvider : IconProvider() {
  override fun getIcon(element: PsiElement, flags: Int) =
    if (element is PsiDirectory && element.hasPackageXml()) Icons.catkin_file else null

  private fun PsiDirectory.hasPackageXml() = files.any { it.name == "package.xml" }
}