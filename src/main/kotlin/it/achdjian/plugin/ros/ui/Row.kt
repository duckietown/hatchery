package it.achdjian.plagin.ros.ui

import com.intellij.BundleBase
import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.ComboBox
import com.intellij.openapi.ui.TextComponentAccessor
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.ui.TextFieldWithHistoryWithBrowseButton
import com.intellij.ui.components.installFileCompletionAndBrowseDialog
import com.intellij.util.ui.SwingHelper
import java.awt.event.ItemEvent
import javax.swing.*
import javax.swing.event.DocumentEvent
import javax.swing.event.DocumentListener

class Row(val text:String? = null) {
    var component: JComponent?=null

    fun checkBox(text: String, actionListener: (event: ItemEvent) -> Unit) {
        val checkBox = JCheckBox(BundleBase.replaceMnemonicAmpersand(text))
        checkBox.addItemListener(actionListener)
        component = checkBox
    }

    fun comboBox(options: Array<String>, selected: String? = null, actionListener: (event: ItemEvent) -> Unit) {
        val comboBox = ComboBox(options)
        comboBox.addItemListener(actionListener)
        if (selected != null)
            comboBox.selectedItem = selected
        component = comboBox
    }

    fun comboBox(options: List<String>, selected: String? = null, actionListener: (event: ItemEvent) -> Unit) {
        comboBox(options.toTypedArray(), selected, actionListener)
    }

    fun textArea(text: String?, changeUpdate: (doc: DocumentEvent?) -> Unit){
        val textArea = JTextArea()
        textArea.text = text
        textArea.document.addDocumentListener(RowDocumentListener(changeUpdate))
        component = textArea
    }

    fun textFieldWithHistoryWithBrowseButton(project: Project?,
                                             value: String?,
                                                 browseDialogTitle: String,
                                                 fileChooserDescriptor: FileChooserDescriptor,
                                                 historyProvider: (() -> List<String>)? = null,
                                                 fileChosen: ((chosenFile: VirtualFile) -> String)? = null){
        val textWithBrowserButton = TextFieldWithHistoryWithBrowseButton()
        val textFieldWithHistory = textWithBrowserButton.childComponent
        textFieldWithHistory.setHistorySize(-1)
        textFieldWithHistory.setMinimumAndPreferredWidth(0)
        if (historyProvider != null) {
            SwingHelper.addHistoryOnExpansion(textFieldWithHistory, historyProvider)
        }
        installFileCompletionAndBrowseDialog(
                project,
                textWithBrowserButton,
                textWithBrowserButton.childComponent.textEditor,
                browseDialogTitle,
                fileChooserDescriptor,
                TextComponentAccessor.TEXT_FIELD_WITH_HISTORY_WHOLE_TEXT,
                fileChosen
        )
        textWithBrowserButton.let{textWithBrowserButton.text = value}
        component = textWithBrowserButton
    }

    fun label(): JLabel {
        if (text != null)
            return JLabel(text)
        else
            return JLabel()
    }
}

class RowDocumentListener(val changeUpdate: (doc: DocumentEvent?) -> Unit ) : DocumentListener{
    override fun changedUpdate(doc: DocumentEvent?) {
        changeUpdate(doc)
    }

    override fun insertUpdate(p0: DocumentEvent?) {
    }

    override fun removeUpdate(p0: DocumentEvent?) {
    }

}