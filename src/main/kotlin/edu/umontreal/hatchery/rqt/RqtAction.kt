package edu.umontreal.hatchery.rqt

import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent

class RqtAction: AnAction {
  constructor()
  constructor(name: String): super("rqt_image_view $name")

  override fun actionPerformed(e: AnActionEvent) {
    // TODO: Fix this hack - unsure how to get action ID, so we use description
    ProcessBuilder(e.presentation.description).start()
  }
}