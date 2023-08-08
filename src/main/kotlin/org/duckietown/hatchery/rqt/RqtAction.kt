package org.duckietown.hatchery.rqt

import com.intellij.openapi.actionSystem.*
import org.duckietown.hatchery.settings.RosConfig

class RqtAction : AnAction() {
  override fun actionPerformed(e: AnActionEvent) {
    e.actionManager.getId(this)?.let {
      RosConfig.rosSettings.localRos.runInBackground(it)
    }
  }
}