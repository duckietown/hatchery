package edu.umontreal.hatchery.util

import com.intellij.ui.layout.Cell
import com.intellij.ui.layout.GrowPolicy.MEDIUM_TEXT
import com.intellij.ui.layout.GrowPolicy.SHORT_TEXT
import javax.swing.JComponent

fun Cell.short(component: JComponent) = component(growPolicy = SHORT_TEXT)
fun Cell.medium(component: JComponent) = component(growPolicy = MEDIUM_TEXT)
