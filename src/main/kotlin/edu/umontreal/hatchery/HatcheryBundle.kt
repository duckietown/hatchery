package edu.umontreal.hatchery

import com.intellij.AbstractBundle
import org.jetbrains.annotations.NonNls
import org.jetbrains.annotations.PropertyKey

class HatcheryBundle private constructor() : AbstractBundle(BUNDLE) {
    companion object {
        fun message(@PropertyKey(resourceBundle = BUNDLE) key: String, vararg params: Any): String {
            return ourInstance.getMessage(key, *params)
        }

        @NonNls private const val BUNDLE = "edu.umontreal.hatchery.HatcheryBundle"
        private val ourInstance = HatcheryBundle()
    }
}