package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosinterface.ROSInterfaceLanguage
import org.jetbrains.annotations.NonNls

class ROSInterfaceElementType(@NonNls debugName: String) : IElementType(debugName, ROSInterfaceLanguage)