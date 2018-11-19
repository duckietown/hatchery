package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.structuralsearch.plugin.ui.TextFieldWithAutoCompletionWithBrowseButton
import com.intellij.ui.layout.panel
import edu.umontreal.hatchery.util.medium
import java.awt.Font
import javax.swing.JTextField
import kotlin.reflect.KProperty

class RosLaunchSettingsEditor(val project: Project, rosPackageName: String, rosLaunchFile: String): SettingsEditor<RosLaunchRunConfig>() {
  private val rosPackageNameTextField = JTextField(rosPackageName)
  private val rosLaunchFileTextField = JTextField(rosLaunchFile)
  private val runCommandTextField = JTextField(rosLaunchFile)
    .apply { font = Font("monospaced", font.style, font.size) }
  private val rosLaunchDestinationField = JTextField(rosLaunchFile)
  private val rosLaunchDestinationPathField = JTextField(rosLaunchFile)

  private var rosPackageName by rosPackageNameTextField
  private var rosLaunchFile by rosLaunchFileTextField
  private var runCommand by runCommandTextField
  private var rosLaunchDestination by rosLaunchDestinationField
  private var rosLaunchDestinationPath by rosLaunchDestinationPathField

  override fun createEditor() = panel {
    row("Package name:") { medium(rosPackageNameTextField) }
    row("Launch file:") { medium(rosLaunchFileTextField) }
    row("Run command:") { medium(runCommandTextField) }
    row("Destination: ") { medium(rosLaunchDestinationField) }
    row("Destination path: ") { medium(rosLaunchDestinationPathField) }
  }

  override fun applyEditorTo(config: RosLaunchRunConfig) {
    config.rosPackageName = rosPackageName
    config.rosLaunchFileName = rosLaunchFile
    config.runCommand = runCommand
    config.remoteAddress = rosLaunchDestination
  }

  override fun resetEditorFrom(config: RosLaunchRunConfig) {
    rosPackageName = config.rosPackageName
    rosLaunchFile = config.rosLaunchFileName
    runCommand = config.runCommand
    rosLaunchDestination = config.rosLaunchFileName
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun TextFieldWithAutoCompletionWithBrowseButton.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun TextFieldWithAutoCompletionWithBrowseButton.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)

  private operator fun JTextField.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun JTextField.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)
}