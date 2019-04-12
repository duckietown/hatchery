package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.TextBrowseFolderListener
import com.intellij.openapi.ui.TextFieldWithBrowseButton
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.ui.components.JBTextField
import com.intellij.ui.layout.panel
import edu.umontreal.hatchery.roslaunch.RosLaunchFileType
import edu.umontreal.hatchery.rosmanifest.RosManifestFileType
import edu.umontreal.hatchery.util.medium
import java.awt.Font
import javax.swing.text.JTextComponent
import kotlin.reflect.KProperty

class RosLaunchSettingsEditor(val project: Project, rosPackageName: String, rosLaunchFile: String) : SettingsEditor<RosLaunchRunConfig>() {
  private val rosPackageBrowseFolderListener = TextBrowseFolderListener(
    object : FileChooserDescriptor(false, true, false, false, false, false) {
      override fun isFileVisible(vf: VirtualFile?, showHidden: Boolean) =
        super.isFileVisible(vf, showHidden) || vf?.fileType === RosManifestFileType

      override fun isFileSelectable(file: VirtualFile?) =
        super.isFileSelectable(file) && file?.isDirectory ?: false &&
          file?.children?.any { it.name == "package.xml" } ?: false
    })

  private val rosPackageNameTextField =
    TextFieldWithBrowseButton(JBTextField(rosPackageName))
      .apply { addBrowseFolderListener(rosPackageBrowseFolderListener) }

  private val rosLaunchBrowseFolderListener = TextBrowseFolderListener(
    object : FileChooserDescriptor(true, false, false, false, false, false) {
      override fun isFileSelectable(file: VirtualFile?) =
        super.isFileSelectable(file) && file?.fileType === RosLaunchFileType
    })

  private val rosLaunchFileTextField: TextFieldWithBrowseButton =
    TextFieldWithBrowseButton(JBTextField(rosLaunchFile))
      .apply { addBrowseFolderListener(rosLaunchBrowseFolderListener) }

  private val rosLaunchOptionsField = JBTextField(rosLaunchFile)
    .apply { font = Font("monospaced", font.style, font.size) }
  private val remoteAddressField = JBTextField(rosLaunchFile)
    .apply { isEnabled = false }
  private val remotePathField = JBTextField(rosLaunchFile)
    .apply { isEnabled = false }
  private val rosLaunchArgsField = JBTextField(rosLaunchFile)
    .apply { font = Font("monospaced", font.style, font.size) }

  private var rosPackageName by rosPackageNameTextField
  private var rosLaunchFile by rosLaunchFileTextField
  private var rosLaunchOptions by rosLaunchOptionsField
  private var rosLaunchArgs by rosLaunchArgsField
  private var remoteAddress by remoteAddressField
  private var remotePath by remotePathField

  override fun createEditor() = panel {
    row("ROS package:") { rosPackageNameTextField(grow) }
    row("Launch file:") { rosLaunchFileTextField(grow) }
    row("Launch flags:") { rosLaunchOptionsField(grow) }
    row("Launch args:") { rosLaunchArgsField(grow) }
    row("Destination: ") { medium(remoteAddressField) }
    row("Destination path: ") { remotePathField(grow) }
  }

  override fun applyEditorTo(config: RosLaunchRunConfig) {
    config.rosPackagePath = rosPackageName
    config.rosLaunchPath = rosLaunchFile
    config.rosLaunchOptions = rosLaunchOptions
    config.rosLaunchArgs = rosLaunchArgs
    config.destinationAddress = remoteAddress
    config.rosLaunchOptions = rosLaunchOptions
  }

  override fun resetEditorFrom(config: RosLaunchRunConfig) {
    rosPackageName = config.rosPackagePath
    rosLaunchFile = config.rosLaunchPath
    rosLaunchOptions = config.rosLaunchOptions
    rosLaunchArgs = config.rosLaunchArgs
    remoteAddress = config.destinationAddress
    remotePath = config.destinationPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun TextFieldWithBrowseButton.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text

  private operator fun TextFieldWithBrowseButton.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)

  private operator fun JTextComponent.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun JTextComponent.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)
}