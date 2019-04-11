package edu.umontreal.hatchery.ros

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import edu.umontreal.hatchery.filesystem.Icons

class LaunchDirIconProvider : IconProvider() {
  override fun getIcon(e: PsiElement, f: Int) = if (isLaunchDir(e) && hasLaunchFiles(e)) Icons.launch_dir else null

  private fun isLaunchDir(element: PsiElement) = element is PsiDirectory && element.name == "launch"

  private fun hasLaunchFiles(element: PsiElement) = (element as PsiDirectory).files.any { it.name.endsWith(".launch") }
}