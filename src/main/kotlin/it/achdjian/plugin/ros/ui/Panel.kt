package it.achdjian.plagin.ros.ui

import com.intellij.ui.IdeBorderFactory
import javax.swing.JPanel

fun panel(title: String? = null, init : LayoutBuilder.()->Any): JPanel {
    val panel = JPanel()
    title?.let{ addTitleBorder(it, panel) }
    val layoutBuilder = LayoutBuilder()
    layoutBuilder.init()
    layoutBuilder.build(panel)
    return panel
}

fun addTitleBorder(title: String, panel: JPanel) {
    val border = IdeBorderFactory.createTitledBorder(title, false)
    panel.border = border
    border.acceptMinimumSize(panel)
}


