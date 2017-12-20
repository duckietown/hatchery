package edu.umontreal.hatchery

import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent

class HatchAction : AnAction() {

    override fun actionPerformed(e: AnActionEvent) {
        println("Hello")
    }
}
