package edu.umontreal.hatchery.psi

import com.intellij.extapi.psi.PsiFileBase
import com.intellij.psi.FileViewProvider
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.rosinterface.ROSInterfaceFileFactory
import edu.umontreal.hatchery.rosinterface.ROSInterfaceLanguage

class ROSInterfaceFile(viewProvider: FileViewProvider) : PsiFileBase(viewProvider, ROSInterfaceLanguage) {
    override fun getFileType() = ROSInterfaceFileFactory.ROSInterfaceFileType

    override fun toString() = "ROSInterface File"

    override fun getIcon(flags: Int) = Icons.robot
}