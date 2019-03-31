package edu.umontreal.hatchery.rqt

import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent
import edu.umontreal.hatchery.settings.RosConfig

object RqtAction: AnAction() {
  override fun actionPerformed(e: AnActionEvent) {
    // TODO: Fix this hack - unsure how to get action ID, so we use description
    RosConfig.settings.localRos.runInBackground(e.presentation.description)
  }
}