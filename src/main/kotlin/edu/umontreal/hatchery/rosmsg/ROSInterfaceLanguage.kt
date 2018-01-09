package edu.umontreal.hatchery.rosmsg

import com.intellij.lang.Language

class ROSInterfaceLanguage private constructor() : Language("Simple") {
    companion object {
        val INSTANCE = ROSInterfaceLanguage()
    }
}