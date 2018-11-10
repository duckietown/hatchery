package edu.umontreal.hatchery.settings

import com.intellij.ui.SeparatorComponent
import com.intellij.ui.layout.panel
import edu.umontreal.hatchery.util.medium
import java.awt.Font
import java.util.*
import javax.swing.JPanel
import javax.swing.JTextField
import javax.swing.text.JTextComponent
import kotlin.reflect.KProperty

class RosSettingsPanel {
  // TODO: Make this into a proper TextFieldWithBrowseButton https://www.jetbrains.org/intellij/sdk/docs/user_interface_components/file_and_class_choosers.html#file-choosers
  private val rosPathField = JTextField()

  init {
    rosPathField.apply { font = Font("monospaced", font.style, font.size) }
  }

  fun prop(s: String) = ResourceBundle.getBundle("HatcheryBundle").getString(s)!!

  internal val rootPanel: JPanel = panel {
    noteRow(prop("rosInstallationPathHeading")) { SeparatorComponent() }
    row { medium(rosPathField) }
  }

  internal var rosPath by rosPathField

  fun reset(settings: RosSettings) {
    rosPath = settings.rosPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun JTextComponent.getValue(a: RosSettingsPanel, p: KProperty<*>) = text
  private operator fun JTextComponent.setValue(a: RosSettingsPanel, p: KProperty<*>, s: String) = setText(s)
}