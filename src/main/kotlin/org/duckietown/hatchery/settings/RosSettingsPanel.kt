package org.duckietown.hatchery.settings

import com.intellij.openapi.ui.TextFieldWithBrowseButton
import com.intellij.ui.IdeBorderFactory
import com.intellij.ui.components.JBTextField
import com.intellij.ui.layout.panel
import org.duckietown.hatchery.util.medium
import java.awt.Font
import java.util.*
import javax.swing.JPanel
import javax.swing.border.LineBorder
import javax.swing.text.JTextComponent
import kotlin.reflect.KProperty

class RosSettingsPanel {
  private val localRosPathField = JBTextField()
//  private val localRosPackages: JBTable
//    get() = JBTable(object : DefaultTableModel(
//      RosConfig.settings.localRos.packages.map { arrayOf(it.key, it.value) }.toTypedArray(),
//      arrayOf("Package", "Path")
//    ) {
//      override fun isCellEditable(row: Int, column: Int) = false
//    })

  private val rosLaunchOptionsField = JBTextField()
    .apply { font = Font("monospaced", font.style, font.size) }
  private val remoteAddressField = JBTextField()
  private val remoteRosPathField = JBTextField()
  private val remoteRunCommandField = JBTextField()
    .apply { font = Font("monospaced", font.style, font.size) }
  private val sshCredentialsPathField = JBTextField()

  init {
    listOf(remoteAddressField,
      remoteRosPathField,
      remoteRunCommandField,
      sshCredentialsPathField)
      .forEach { it.isEnabled = false }
  }

  private fun prop(s: String) = ResourceBundle.getBundle("HatcheryBundle").getString(s)!!

  internal val rootPanel: JPanel
    get() = panel {
      row(prop("rosInstallationHeading")) {}
      row("ROS installation path:") { localRosPathField(grow) }
      row("Default ROS Launch options:") { medium(rosLaunchOptionsField) }
//      row("Installed ROS packages") {}
//      row { JBScrollPane(localRosPackages)(growX) }
      row(prop("remoteSettingsHeading")) { LineBorder.createGrayLineBorder() }
      row("Remote address:") { medium(remoteAddressField) }
      row("Remote ROS installation Path:") { remoteRosPathField(grow) }
      row("Remote ROS run command:") { medium(remoteRunCommandField) }
      row(prop("sshSettingsHeading")) {}
      row("Path to SSH credentials file:") { sshCredentialsPathField(grow) }
    }.also { it.border = IdeBorderFactory.createTitledBorder("Test") }

  internal var localRosPath by localRosPathField
  internal var defaultRosLaunchOptions by rosLaunchOptionsField
  internal var remoteAddress by remoteAddressField
  internal var remoteRosPath by remoteRosPathField
  internal var remoteRunCommand by remoteRunCommandField
  internal var sshCredentialsPath by sshCredentialsPathField

  fun reset(settings: RosSettings) {
    localRosPath = settings.localRosPath
    defaultRosLaunchOptions = settings.defaultRosLaunchOptions
    remoteRosPath = settings.remoteRosPath
    remoteAddress = settings.remoteAddress
    remoteRunCommand = settings.remoteRunCommand
    sshCredentialsPath = settings.sshCredentialsPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun JTextComponent.getValue(a: RosSettingsPanel, p: KProperty<*>) = text
  private operator fun JTextComponent.setValue(a: RosSettingsPanel, p: KProperty<*>, s: String) = setText(s)

  private operator fun TextFieldWithBrowseButton.getValue(a: RosSettingsPanel, p: KProperty<*>) = text
  private operator fun TextFieldWithBrowseButton.setValue(a: RosSettingsPanel, p: KProperty<*>, s: String) = setText(s)
}