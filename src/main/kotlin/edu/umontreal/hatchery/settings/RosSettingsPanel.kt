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
  private val localRosPathField = JTextField()
  private val localRunCommandField = JTextField()
    .apply { font = Font("monospaced", font.style, font.size) }
  private val remoteAddressField = JTextField()
  private val remoteRosPathField = JTextField()
  private val remoteRunCommandField = JTextField()
    .apply { font = Font("monospaced", font.style, font.size) }
  private val sshCredentialsPathField = JTextField()

  fun prop(s: String) = ResourceBundle.getBundle("HatcheryBundle").getString(s)!!

  internal val rootPanel: JPanel = panel {
    noteRow(prop("rosInstallationHeading")) { SeparatorComponent() }
    row("ROS installation path:") { medium(localRosPathField) }
    row("ROS run command path:") { medium(localRunCommandField) }
    noteRow(prop("remoteSettingsHeading")) { SeparatorComponent() }
    row("Remote address: ") { medium(remoteAddressField) }
    row("Remote ROS installation Path:") { medium(remoteRosPathField) }
    noteRow(prop("sshSettingsHeading")) { SeparatorComponent() }
    row("Path to SSH credentials file:") { medium(sshCredentialsPathField) }
  }

  internal var localRosPath by localRosPathField
  internal var localRunCommand by localRunCommandField
  internal var remoteAddress by remoteAddressField
  internal var remoteRosPath by remoteRosPathField
  internal var remoteRunCommand by localRunCommandField
  internal var sshCredentialsPath by sshCredentialsPathField

  fun reset(settings: RosSettings) {
    localRosPath = settings.localRosPath
    localRunCommand = settings.localRunCommand
    localRosPath = settings.localRosPath
    remoteRosPath = settings.remoteRosPath
    remoteAddress = settings.remoteAddress
    remoteRunCommand = settings.remoteRunCommand
    sshCredentialsPath = settings.sshCredentialsPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun JTextComponent.getValue(a: RosSettingsPanel, p: KProperty<*>) = text

  private operator fun JTextComponent.setValue(a: RosSettingsPanel, p: KProperty<*>, s: String) = setText(s)
}