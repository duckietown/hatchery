package edu.umontreal.hatchery.psi

import com.intellij.extapi.psi.PsiFileBase
import com.intellij.psi.FileViewProvider
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.rosinterface.RosInterfaceFileType
import edu.umontreal.hatchery.rosinterface.RosInterfaceLanguage

class RosInterfaceFile(viewProvider: FileViewProvider) : PsiFileBase(viewProvider, RosInterfaceLanguage) {
  override fun getFileType() = RosInterfaceFileType

  override fun toString() = "RosInterface File"

  override fun getIcon(flags: Int) = Icons.robot
}