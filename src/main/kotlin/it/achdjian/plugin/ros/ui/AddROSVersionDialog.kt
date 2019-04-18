package it.achdjian.plugin.ros.ui

import com.intellij.openapi.fileChooser.FileChooserDescriptorFactory
import com.intellij.openapi.ui.DialogWrapper
import com.intellij.openapi.ui.TextComponentAccessor
import com.intellij.ui.TextFieldWithHistoryWithBrowseButton
import com.intellij.ui.components.installFileCompletionAndBrowseDialog
import com.intellij.util.ui.JBUI
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.event.KeyEvent
import java.awt.event.KeyListener
import javax.swing.*

class NameKeyListener(val action :() ->Unit) : KeyListener {
    override fun keyTyped(p0: KeyEvent?) = action()
    override fun keyPressed(p0: KeyEvent?) = action()
    override fun keyReleased(p0: KeyEvent?) = action()
}

class AddROSVersionDialog(startName:String, startPath:String) : DialogWrapper(null, true) {
    constructor():this("","")
    private var versionName = JTextField()
    private val versionPath = TextFieldWithHistoryWithBrowseButton()
    val name: String
        get() = versionName.text?.let { it } ?: "no name"
    val path: String
        get() = versionPath.text?.let { it } ?: ""

    init {
        title = "Add ROS version"
        versionName.text = startName
        versionPath.text = startPath
        init()

        isOKActionEnabled = false
    }

    override fun createCenterPanel(): JComponent? {
        val layout = GridBagLayout()
        val mainPanel = JPanel(layout)

        val version = JLabel("ROS version name")

        val pathLabel = JLabel("Path")

        versionName.addKeyListener(NameKeyListener{
            if (versionPath.text.isNotEmpty() && versionName.text.isNotEmpty())
                isOKActionEnabled=true
        })


        versionPath.childComponent.textEditor.addKeyListener(NameKeyListener{
            if (versionPath.text.isNotEmpty() && versionName.text.isNotEmpty())
                isOKActionEnabled=true
        })

        val editor = versionPath.childComponent.textEditor

        installFileCompletionAndBrowseDialog(
                null,
                versionPath,
                editor,
                "ROS version versionPath",
                FileChooserDescriptorFactory.createSingleFolderDescriptor(),
                TextComponentAccessor.TEXT_FIELD_WITH_HISTORY_WHOLE_TEXT) {
            if (it.path.isNotEmpty() && versionName.text.isNotEmpty())
                isOKActionEnabled = true
            it.path

        }

        val c = GridBagConstraints()
        c.fill = GridBagConstraints.NONE
        c.insets = JBUI.insets(2)
        c.gridx = 0
        c.gridy = 0
        mainPanel.add(version, c)
        c.fill = GridBagConstraints.HORIZONTAL
        c.gridx = 1
        c.weightx = 1.0
        mainPanel.add(versionName, c)
        c.fill = GridBagConstraints.NONE
        c.gridx = 0
        c.gridy = 1
        c.weightx = 0.0
        mainPanel.add(pathLabel, c)
        c.fill = GridBagConstraints.HORIZONTAL
        c.gridx = 1
        c.weightx = 1.0
        mainPanel.add(versionPath, c)

        return mainPanel
    }
}