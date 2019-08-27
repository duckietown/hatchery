package it.achdjian.plugin.ros.ui

import com.intellij.ui.ColoredListCellRenderer
import it.achdjian.plugin.ros.data.RosPackage
import javax.swing.JList

class RosPackageListCellRenderer : ColoredListCellRenderer<RosPackage>() {
    override fun customizeCellRenderer(list: JList<out RosPackage>, value: RosPackage?, index: Int, selected: Boolean, hasFocus: Boolean) {
        value?.let { append(it.name) }
    }

}