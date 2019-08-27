package it.achdjian.plagin.ros.ui

import javax.swing.JComponent
import javax.swing.JPanel

class LayoutBuilder {
    private val rows = ArrayList<Row>()

    fun row(text:String?=null,init: Row.() -> Unit){
        val row = Row(text)
        row.init()
        rows.add(row)
    }

    fun row(text:String?=null,component: JComponent){
        val row = Row(text)
        row.component = component
        rows.add(row)
    }

    fun build(panel: JPanel) {
        var colSize=1

        if (rows.any())
            colSize = 2

        panel.layout = GridLayout2(rows.size, colSize)

        if (colSize==1) {
            rows.forEach { panel.add(it.component) }
        }else {
            rows.forEach {
                panel.add(it.label())
                panel.add(it.component)
            }
        }
    }
}