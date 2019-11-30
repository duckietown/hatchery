package org.duckietown.hatchery.achdjian.ui

import com.intellij.openapi.util.IconLoader
import com.intellij.ui.*
import org.duckietown.hatchery.achdjian.data.RosVersionImpl
import javax.swing.JList

class RosVersionListCellRenderer : ColoredListCellRenderer<RosVersionImpl>() {
    override fun customizeCellRenderer(list: JList<out RosVersionImpl>, value: RosVersionImpl?, index: Int, selected: Boolean, hasFocus: Boolean) {
        value?.let {
            icon = IconLoader.findIcon("/org/duckietown/hatchery/icons/ros.svg")
            append(it.name)
            append(it.path, SimpleTextAttributes.GRAYED_SMALL_ATTRIBUTES)
        }
    }
}