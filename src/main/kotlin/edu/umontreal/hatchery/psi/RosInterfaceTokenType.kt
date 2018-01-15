package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosinterface.RosInterfaceLanguage
import org.jetbrains.annotations.NonNls

class RosInterfaceTokenType(@NonNls debugName: String) : IElementType(debugName, RosInterfaceLanguage) {
  override fun toString() = "RosInterfaceTokenType." + super.toString()
}