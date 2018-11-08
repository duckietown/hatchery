package edu.umontreal.hatchery.settings

import com.intellij.ui.SeparatorComponent
import com.intellij.ui.layout.Cell
import com.intellij.ui.layout.GrowPolicy.MEDIUM_TEXT
import com.intellij.ui.layout.GrowPolicy.SHORT_TEXT
import com.intellij.ui.layout.panel
import java.awt.Font
import java.util.*
import javax.swing.JComponent
import javax.swing.JPanel
import javax.swing.JTextField
import javax.swing.text.JTextComponent
import kotlin.reflect.KProperty

class RosSettingsPanel {
  private val rosPathField = JTextField()

  init {
    rosPathField.apply { font = Font("monospaced", font.style, font.size) }
  }

  fun prop(s: String) = ResourceBundle.getBundle("HatcheryBundle").getString(s)!!

  internal val rootPanel: JPanel = panel {
    fun Cell.short(component: JComponent) = component(growPolicy = SHORT_TEXT)
    fun Cell.medium(component: JComponent) = component(growPolicy = MEDIUM_TEXT)

    noteRow(prop("rosInstallationPathHeading")) { SeparatorComponent() }
    row { medium(rosPathField) }
  }

  internal var rosPath by rosPathField

  fun reset(settings: RosSettings) {
    rosPath = settings.rosPath
  }

  // Removal pending support for https://youtrack.jetbrains.com/issue/KT-8575
  private operator fun JTextComponent.getValue(a: RosSettingsPanel, p: KProperty<*>) = text.toLowerCase()

  private operator fun JTextComponent.setValue(a: RosSettingsPanel, p: KProperty<*>, s: String) {
    text = s
  }
}