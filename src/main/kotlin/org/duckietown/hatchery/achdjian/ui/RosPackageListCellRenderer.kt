package org.duckietown.hatchery.achdjian.ui

import com.intellij.ui.ColoredListCellRenderer
import org.duckietown.hatchery.achdjian.data.RosPackage
import javax.swing.JList

class RosPackageListCellRenderer : ColoredListCellRenderer<RosPackage>() {
    override fun customizeCellRenderer(list: JList<out RosPackage>, value: RosPackage?, index: Int, selected: Boolean, hasFocus: Boolean) {
        value?.let { append(it.name) }
    }

}