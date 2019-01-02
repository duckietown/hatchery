package edu.umontreal.hatchery.roslaunch

import com.intellij.icons.AllIcons
import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent

class RosLaunchRunTargetAction: AnAction {
  constructor()
  constructor(name: String): super("roslaunch $name", "roslaunch $name", AllIcons.RunConfigurations.TestState.Run)

  override fun actionPerformed(e: AnActionEvent) {
    TODO("not implemented")
  }
}