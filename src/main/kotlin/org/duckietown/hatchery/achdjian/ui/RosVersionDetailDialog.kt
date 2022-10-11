package org.duckietown.hatchery.achdjian.ui

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.ui.DialogWrapper
import com.intellij.ui.*
import com.intellij.ui.components.JBList
import org.duckietown.hatchery.achdjian.data.RosVersionImpl
import org.duckietown.hatchery.achdjian.data.*
import java.awt.Dimension
import javax.swing.*

class RosVersionDetailDialog : DialogWrapper(null, true) {
    private var versionList = JBList<RosVersionImpl>()
    private var mainPanel: JPanel? = null

    init {
        title = "ROS VERSION"
        init()
    }

    override fun createCenterPanel(): JComponent {
        val decorator = ToolbarDecorator
                .createDecorator(versionList)
                .disableUpDownActions()
                .setAddAction { addSdk() }
                .setEditAction { editSdk() }
                .setRemoveAction { removeSdk() }
        decorator.setPreferredSize(Dimension(600, 500))
        val panel = decorator.createPanel()
        mainPanel = panel
        versionList.cellRenderer = RosVersionListCellRenderer()
        versionList.selectionMode = ListSelectionModel.SINGLE_SELECTION
        ListSpeedSearch(versionList)
        refreshVersionList()
        return panel
    }

    private fun refreshVersionList() {
        versionList.removeAll()
        val state = getRosEnvironment()
        versionList.clearSelection()
        versionList.model = CollectionListModel(state.versions)
    }

    private fun removeSdk() {
        val selectedVersion = versionList.selectedValue
        selectedVersion?.let {
            val customVersion =
                ApplicationManager.getApplication().getComponent(RosCustomVersion::class.java)
            if (customVersion.contains(it)) {
                customVersion.remove(it)
            }
            val state = getRosEnvironment()
            if (state.isDefaultVersion(it.name)) {
                customVersion.removeDefault(it.name)
            }
            state.remove(it)
            refreshVersionList()
            isOKActionEnabled = true
        }
    }

    private fun addSdk() {
        ApplicationManager.getApplication().invokeLater {
            val addDialog = AddROSVersionDialog()
            addDialog.show()

            if (addDialog.isOK) {
                val customVersion = ApplicationManager.getApplication().getComponent(RosCustomVersion::class.java)
                customVersion.versions[addDialog.name] = addDialog.path
                val state = getRosEnvironment()
                state.add(RosVersionImpl(addDialog.path, addDialog.name))
                refreshVersionList()
                isOKActionEnabled = true
            }
        }
    }

    private fun editSdk() {
        ApplicationManager.getApplication().invokeLater {
            versionList.selectedValue?.let {
                val addDialog = AddROSVersionDialog(it.name, it.path)
                addDialog.show()

                if (addDialog.isOK) {
                    val state = getRosEnvironment()
                    val customVersion = ApplicationManager.getApplication().getComponent(RosCustomVersion::class.java)
                    if (state.isDefaultVersion(it.name)) {
                        customVersion.removeDefault(it.name)
                    }
                    state.remove(it)
                    customVersion.versions[addDialog.name] = addDialog.path
                    state.add(RosVersionImpl(addDialog.path, addDialog.name))
                    refreshVersionList()
                    isOKActionEnabled = true
                }
            }
        }
    }
}
