package edu.umontreal.hatchery.rosinterface

import com.intellij.lexer.FlexAdapter
import edu.umontreal.hatchery.ROSInterfaceLexer

class ROSInterfaceLexerAdapter : FlexAdapter(ROSInterfaceLexer(null))
