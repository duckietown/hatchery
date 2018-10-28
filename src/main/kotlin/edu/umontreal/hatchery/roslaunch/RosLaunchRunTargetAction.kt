package edu.umontreal.hatchery.roslaunch

import com.intellij.icons.AllIcons
import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent

class RosLaunchRunTargetAction : AnAction {
  constructor(name: String) : super("roslaunch $name", "roslaunch $name", AllIcons.General.Run)

  override fun actionPerformed(e: AnActionEvent?) {
    TODO("not yet implemented") //To change body of created functions use File | Settings | File Templates.
  }
}