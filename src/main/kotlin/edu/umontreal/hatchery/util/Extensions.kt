package edu.umontreal.hatchery.util

import com.intellij.ui.layout.Cell
import com.intellij.ui.layout.GrowPolicy
import javax.swing.JComponent

fun Cell.short(component: JComponent) = component(growPolicy = GrowPolicy.SHORT_TEXT)
fun Cell.medium(component: JComponent) = component(growPolicy = GrowPolicy.MEDIUM_TEXT)
