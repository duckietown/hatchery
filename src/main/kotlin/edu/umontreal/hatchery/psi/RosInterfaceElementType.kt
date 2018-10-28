package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosinterface.RosInterfaceLanguage
import org.jetbrains.annotations.NonNls

class RosInterfaceElementType : IElementType {
  constructor(@NonNls debugName: String) : super(debugName, RosInterfaceLanguage)
}