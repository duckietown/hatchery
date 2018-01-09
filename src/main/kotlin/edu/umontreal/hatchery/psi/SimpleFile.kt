package edu.umontreal.hatchery.psi

import com.intellij.extapi.psi.PsiFileBase
import com.intellij.openapi.fileTypes.FileType
import com.intellij.psi.FileViewProvider
import edu.umontreal.hatchery.rosmsg.ROSInterfaceFileFactory
import edu.umontreal.hatchery.rosmsg.ROSInterfaceLanguage

import javax.swing.*

class SimpleFile(viewProvider: FileViewProvider) : PsiFileBase(viewProvider, ROSInterfaceLanguage.INSTANCE) {

    override fun getFileType(): FileType {
        return ROSInterfaceFileFactory.ROSInterfaceFileType
    }

    override fun toString(): String {
        return "Simple File"
    }

    override fun getIcon(flags: Int): Icon? {
        return super.getIcon(flags)
    }
}