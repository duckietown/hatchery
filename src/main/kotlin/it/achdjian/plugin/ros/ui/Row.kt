package it.achdjian.plugin.ros.ui

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
import javax.swing.JCheckBox
import javax.swing.JComponent
import javax.swing.JLabel
import javax.swing.JTextArea
import javax.swing.event.DocumentEvent
import javax.swing.event.DocumentListener

class Row(val text: String? = null) {
  var component: JComponent? = null

  fun checkBox(text: String, actionListener: (event: ItemEvent) -> Unit) {
    component = JCheckBox(BundleBase.replaceMnemonicAmpersand(text)).apply {
      addItemListener(actionListener)
    }
  }

  fun comboBox(options: List<String>,
               selected: String? = null,
               actionListener: (event: ItemEvent) -> Unit) {
    component = ComboBox(options.toTypedArray()).apply {
      addItemListener(actionListener)
      if (selected != null) selectedItem = selected
    }
  }

  fun textArea(text: String?, changeUpdate: (doc: DocumentEvent?) -> Unit) {
    val textArea = JTextArea()
    textArea.text = text
    textArea.document.addDocumentListener(RowDocumentListener(changeUpdate))
    component = textArea
  }

  fun textFieldWithHistoryWithBrowseButton(
    project: Project?,
    value: String?,
    browseDialogTitle: String,
    fileChooserDescriptor: FileChooserDescriptor,
    historyProvider: (() -> List<String>)? = null,
    fileChosen: ((chosenFile: VirtualFile) -> String)? = null
  ) {
    val textWithBrowserButton = TextFieldWithHistoryWithBrowseButton()
    textWithBrowserButton.childComponent.apply {
      setHistorySize(-1)
      setMinimumAndPreferredWidth(0)
      if (historyProvider != null)
        SwingHelper.addHistoryOnExpansion(this, historyProvider)
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
    textWithBrowserButton.let { textWithBrowserButton.text = value }
    component = textWithBrowserButton
  }

  fun label() = if (text != null) JLabel(text) else JLabel()
}

class RowDocumentListener(val changeUpdate: (doc: DocumentEvent?) -> Unit) : DocumentListener {
  override fun changedUpdate(doc: DocumentEvent?) = changeUpdate(doc)

  override fun insertUpdate(p0: DocumentEvent?) {}

  override fun removeUpdate(p0: DocumentEvent?) {}
}