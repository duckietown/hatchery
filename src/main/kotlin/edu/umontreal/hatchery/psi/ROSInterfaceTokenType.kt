package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosinterface.ROSInterfaceLanguage
import org.jetbrains.annotations.NonNls

class ROSInterfaceTokenType(@NonNls debugName: String) : IElementType(debugName, ROSInterfaceLanguage) {
    override fun toString() = "ROSInterfaceTokenType." + super.toString()
}