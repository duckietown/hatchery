package edu.umontreal.hatchery.roslaunch.runconfig

import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.ui.layout.panel


class RosLaunchSettingsEditor(project: Project) : SettingsEditor<RosLaunchRunConfiguration>() {
  private val launchSettingsPanel = panel {
    noteRow("Testing Kotlin Panel")
  }

  override fun resetEditorFrom(s: RosLaunchRunConfiguration) {}

  override fun createEditor() = launchSettingsPanel

  override fun applyEditorTo(configuration: RosLaunchRunConfiguration) {}
}
