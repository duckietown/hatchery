package org.duckietown.hatchery.achdjian.ui

import com.intellij.openapi.fileChooser.*
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.TextComponentAccessor
import com.intellij.openapi.ui.TextComponentAccessors
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.ui.TextFieldWithHistoryWithBrowseButton
import com.intellij.ui.components.installFileCompletionAndBrowseDialog
import com.intellij.util.ui.SwingHelper

class LauncherFileChooser(browseDialogTitle: String,
                          project: Project? = null,
                          fileChooserDescriptor: FileChooserDescriptor = FileChooserDescriptorFactory.createSingleFileNoJarsDescriptor(),
                          historyProvider: (() -> List<String>)? = null,
                          fileChosen: ((chosenFile: VirtualFile) -> String)? = null) : TextFieldWithHistoryWithBrowseButton() {
    init {
        childComponent.setHistorySize(-1)
        childComponent.setMinimumAndPreferredWidth(0)
        if (historyProvider != null) {
            SwingHelper.addHistoryOnExpansion(childComponent, historyProvider)
        }
        installFileCompletionAndBrowseDialog(
                project = project,
                component = this,
                textField = childComponent.textEditor,
                browseDialogTitle = browseDialogTitle,
                fileChooserDescriptor = fileChooserDescriptor,
                textComponentAccessor = TextComponentAccessors.TEXT_FIELD_WITH_HISTORY_WHOLE_TEXT,
                fileChosen = fileChosen
        )
    }
}
