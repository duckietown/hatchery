package edu.umontreal.hatchery.psi

import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.rosmsg.ROSInterfaceLanguage
import org.jetbrains.annotations.NonNls

class SimpleElementType(@NonNls debugName: String) : IElementType(debugName, ROSInterfaceLanguage.INSTANCE)