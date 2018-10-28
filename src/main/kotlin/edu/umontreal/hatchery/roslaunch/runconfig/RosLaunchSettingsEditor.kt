package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.options.SettingsEditor
import com.intellij.ui.layout.panel


object RosLaunchSettingsEditor : SettingsEditor<RosLaunchRunConfiguration>() {
  override fun resetEditorFrom(s: RosLaunchRunConfiguration) {}

  override fun createEditor() = panel {
    noteRow("roslaunch run configuration")
  }

  override fun applyEditorTo(configuration: RosLaunchRunConfiguration) {}
}
