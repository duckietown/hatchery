package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosmsg.ROSInterfaceLanguage
import org.jetbrains.annotations.NonNls

class SimpleTokenType(@NonNls debugName: String) : IElementType(debugName, ROSInterfaceLanguage.INSTANCE) {

    override fun toString(): String {
        return "SimpleTokenType." + super.toString()
    }
}