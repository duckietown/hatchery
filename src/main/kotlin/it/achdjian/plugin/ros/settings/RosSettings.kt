package it.achdjian.plugin.ros.settings

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.components.BaseComponent
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.options.Configurable
import com.intellij.openapi.ui.ComboBox
import com.intellij.openapi.ui.FixedSizeButton
import com.intellij.openapi.util.IconLoader
import com.intellij.ui.ComboboxSpeedSearch
import com.intellij.ui.table.JBTable
import com.intellij.util.ui.JBUI
import it.achdjian.plugin.ros.data.getRosEnvironment
import it.achdjian.plugin.ros.ui.RosTablePackageModel
import it.achdjian.plugin.ros.ui.RosVersionDetailDialog
import it.achdjian.plugin.ros.ui.VersionSelector
import java.awt.Dimension
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import javax.swing.JComponent
import javax.swing.JLabel
import javax.swing.JPanel
import javax.swing.JScrollPane


class RosSettings : BaseComponent, Configurable {
    companion object {
        private val LOG = Logger.getInstance(RosSettings::class.java)
    }

    private var modified = false
    private val versionSelector = ComboBox<String>()
    override fun isModified() = modified

    private val model = RosTablePackageModel()

    override fun initComponent() {
        updateVersionSelector()
    }

    override fun apply() {
    }

    override fun createComponent(): JComponent {
        val layout = GridBagLayout()
        val mainPanel = JPanel(layout)
        val version = JLabel("ROS version")
        val emptyLabel = JLabel("  ")
        updateVersionSelector()
        ComboboxSpeedSearch(versionSelector)
        versionSelector.putClientProperty(VersionSelector.TABLE_CELL_EDITOR, true)
        versionSelector.addActionListener {
            updateTable()
        }
        val preferredSize = versionSelector.preferredSize


        val detailsButton = FixedSizeButton()
        detailsButton.icon = IconLoader.findIcon("/icons/ros.svg")
        detailsButton.preferredSize = Dimension(preferredSize.height, preferredSize.height)
        detailsButton.addActionListener {
            ApplicationManager.getApplication().invokeLater {
                val dialog = RosVersionDetailDialog()
                dialog.show()
                if (dialog.isOK) {
                    modified = true
                    updateVersionSelector()
                }
            }
        }

        val packageTable = JBTable(model)
        updateTable()


        val c = GridBagConstraints()
        c.fill = GridBagConstraints.HORIZONTAL
        c.insets = JBUI.insets(2)
        c.gridx = 0
        c.gridy = 0
        mainPanel.add(version, c)
        c.gridx = 1
        c.gridy = 0
        c.weightx = 0.1
        mainPanel.add(versionSelector, c)
        c.insets = JBUI.insets(2, 0, 2, 2)
        c.gridx = 2
        c.gridy = 0
        c.weightx = 0.0
        mainPanel.add(detailsButton, c)

        c.insets = JBUI.insets(2, 2, 0, 2)
        c.gridx = 0
        ++c.gridy
        c.gridwidth = 3
        c.weightx = 0.0
        mainPanel.add(emptyLabel, c)
        c.gridx = 0
        ++c.gridy
        c.weighty = 1.0
        c.gridwidth = 3
        c.gridheight = -1
        c.fill = 1
        mainPanel.add(JScrollPane(packageTable), c)

        return mainPanel
    }

    private fun updateTable() {
        val state = getRosEnvironment()
        LOG.trace("selected version: ${versionSelector.selectedItem}")
        state.versions.forEach { rosVersion ->
            if (rosVersion.name == versionSelector.selectedItem) {
                model.updateVersions(rosVersion)
            }
        }
    }

    private fun updateVersionSelector() {
        val state = getRosEnvironment()
        versionSelector.removeAllItems()
        versionSelector
        state.versions.forEach {
            versionSelector.addItem(it.name)
        }
    }


    override fun getDisplayName() = "ROS"
}
