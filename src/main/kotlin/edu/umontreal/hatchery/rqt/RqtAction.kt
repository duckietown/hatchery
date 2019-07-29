package edu.umontreal.hatchery.rqt

import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent
import edu.umontreal.hatchery.settings.RosConfig

class RqtAction : AnAction() {
  override fun actionPerformed(e: AnActionEvent) {
    RosConfig.settings.localRos.runInBackground(e.actionManager.getId(this))
  }
}