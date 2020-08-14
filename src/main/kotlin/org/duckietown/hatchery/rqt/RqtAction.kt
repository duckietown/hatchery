package org.duckietown.hatchery.rqt

import com.intellij.openapi.actionSystem.*
import org.duckietown.hatchery.settings.RosConfig

class RqtAction : AnAction() {
  override fun actionPerformed(e: AnActionEvent) {
    RosConfig.settings.localRos.runInBackground(e.actionManager.getId(this))
  }
}