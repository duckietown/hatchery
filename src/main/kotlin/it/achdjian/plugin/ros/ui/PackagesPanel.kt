package it.achdjian.plugin.ros.ui

import com.intellij.ui.IdeBorderFactory
import com.intellij.ui.components.CheckBox
import it.achdjian.plugin.ros.data.RosPackage
import java.util.stream.Collectors
import javax.swing.BoxLayout
import javax.swing.JCheckBox
import javax.swing.JPanel

class PackagesPanel() : JPanel() {
    val status = HashMap<String, Boolean>()

    init {
        layout = BoxLayout(this, BoxLayout.Y_AXIS)
        border = IdeBorderFactory.createTitledBorder("Packages")
    }

    fun setPackages(data: List<RosPackage>) {
        removeAll()
        data.forEach {
            val checkBox = CheckBox(it.name)
            checkBox.addActionListener { action ->
                val source = action.source as JCheckBox
                status[it.name] = source.isSelected
            }
            add(checkBox)
        }
        revalidate()
    }

    fun selected():List<String>  = status.entries.stream().filter { it.value }.map { it.key }.collect(Collectors.toList())

}