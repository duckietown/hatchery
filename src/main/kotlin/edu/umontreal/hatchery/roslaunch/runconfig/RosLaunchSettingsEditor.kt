package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.structuralsearch.plugin.ui.TextFieldWithAutoCompletionWithBrowseButton
import com.intellij.ui.layout.panel
import edu.umontreal.hatchery.util.medium
import javax.swing.JTextField
import kotlin.reflect.KProperty

class RosLaunchSettingsEditor(val project: Project, rosPackageName: String, rosLaunchFile: String) : SettingsEditor<RosLaunchRunConfiguration>() {
  private val rosPackageNameTextField = JTextField(rosPackageName)
  private val rosLaunchFileTextField = JTextField(rosLaunchFile)
  private var rosPackageName by rosPackageNameTextField
  private var rosLaunchFile by rosLaunchFileTextField

  override fun createEditor() = panel {
    row("Package name:") { medium(rosPackageNameTextField) }
    row("Launch file:") { medium(rosLaunchFileTextField) }
  }

  override fun applyEditorTo(config: RosLaunchRunConfiguration) {
    config.rosPackageName = rosPackageName
    config.rosLaunchFileName = rosLaunchFile
  }

  override fun resetEditorFrom(config: RosLaunchRunConfiguration) {
    rosPackageName = config.rosPackageName
    rosLaunchFile = config.rosLaunchFileName
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun TextFieldWithAutoCompletionWithBrowseButton.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun TextFieldWithAutoCompletionWithBrowseButton.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)

  private operator fun JTextField.getValue(a: RosLaunchSettingsEditor, p: KProperty<*>) = text
  private operator fun JTextField.setValue(a: RosLaunchSettingsEditor, p: KProperty<*>, s: String) = setText(s)
}