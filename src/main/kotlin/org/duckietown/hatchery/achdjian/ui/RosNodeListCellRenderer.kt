package org.duckietown.hatchery.achdjian.ui

import com.intellij.ui.ColoredListCellRenderer
import org.duckietown.hatchery.achdjian.data.RosNode
import javax.swing.JList

class RosNodeListCellRenderer : ColoredListCellRenderer<RosNode>() {
    override fun customizeCellRenderer(list: JList<out RosNode>, rosNode: RosNode?, index: Int, selected: Boolean, hasFocus: Boolean) {
        rosNode?.let { append(it.name) }
    }
}