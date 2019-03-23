package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.TextBrowseFolderListener
import com.intellij.openapi.ui.TextFieldWithBrowseButton
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.ui.layout.panel
import edu.umontreal.hatchery.roslaunch.RosLaunchFileType
import edu.umontreal.hatchery.rospackage.RosPackageFileType
import edu.umontreal.hatchery.util.medium
import java.awt.Font
import javax.swing.JTextArea
import javax.swing.JTextField
import kotlin.reflect.KProperty

class RosLaunchSettingsEditor(val project: Project, rosPackageName: String, rosLaunchFile: String): SettingsEditor<RosLaunchRunConfig>() {
  private val rosPackageBrowseFolderListener = TextBrowseFolderListener(
    object: FileChooserDescriptor(false, true, false, false, false, false) {
      override fun isFileVisible(vf: VirtualFile?, showHidden: Boolean) =
        super.isFileVisible(vf, showHidden) || vf?.fileType === RosPackageFileType

      override fun isFileSelectable(file: VirtualFile?) =
        super.isFileSelectable(file) && file?.isDirectory ?: false &&
          file?.children?.any { it.name == "package.xml" } ?: false
    })

  private val rosPackageNameTextField =
    TextFieldWithBrowseButton(JTextField(rosPackageName))
      .apply { addBrowseFolderListener(rosPackageBrowseFolderListener) }

  private val rosLaunchBrowseFolderListener = TextBrowseFolderListener(
    object: FileChooserDescriptor(true, false, false, false, false, false) {
      override fun isFileSelectable(file: VirtualFile?) =
        super.isFileSelectable(file) && file?.fileType === RosLaunchFileType
    })

  private val rosLaunchFileTextField =
    TextFieldWithBrowseButton(JTextField(rosLaunchFile))
      .apply { addBrowseFolderListener(rosLaunchBrowseFolderListener) }

  private val runCommandTextArea = JTextArea(rosLaunchFile)
    .apply { font = Font("monospaced", font.style, font.size); lineWrap = true }
  private val rosLaunchDestinationAddressField = JTextField(rosLaunchFile)
  private val rosLaunchDestinationPathField = JTextField(rosLaunchFile)

  private var rosPackageName by rosPackageNameTextField
  private var rosLaunchFile by rosLaunchFileTextField
  private var runCommand by runCommandTextArea
  private var remoteAddress by rosLaunchDestinationAddressField
  private var remoteRosPath by rosLaunchDestinationPathField

  override fun createEditor() = panel {
    row("ROS package:") { rosPackageNameTextField(grow) }
    row("Launch file:") { rosLaunchFileTextField(grow) }
    row("Run command:") { runCommandTextArea(grow) }
    row("Destination: ") { medium(rosLaunchDestinationAddressField) }
    row("Destination path: ") { rosLaunchDestinationPathField(grow) }
  }

  override fun applyEditorTo(config: RosLaunchRunConfig) {
    config.rosPackagePath = rosPackageName
    config.rosLaunchPath = rosLaunchFile
    config.remoteAddress = remoteAddress
  }

  override fun resetEditorFrom(config: RosLaunchRunConfig) {
    rosPackageName = config.rosPackagePath
    rosLaunchFile = config.rosLaunchPath
    remoteAddress = config.remoteAddress
    remoteRosPath = config.remoteRosPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun TextFieldWithBrowseButton.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun TextFieldWithBrowseButton.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)

  private operator fun JTextField.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun JTextField.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)

  private operator fun JTextArea.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun JTextArea.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)
}