package it.achdjian.plagin.ros.ui

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.MessageType
import com.intellij.openapi.wm.ToolWindowId
import com.intellij.openapi.wm.ToolWindowManager

fun showMessage(project: Project, messageType: MessageType, message: String) {
    ApplicationManager.getApplication().invokeLater {
        val toolWindowManager = ToolWindowManager.getInstance(project)
        if (toolWindowManager.canShowNotification(ToolWindowId.RUN)) {
            toolWindowManager.notifyByBalloon(ToolWindowId.DEPENDENCIES, messageType, message, null, null)
        }
    }
}