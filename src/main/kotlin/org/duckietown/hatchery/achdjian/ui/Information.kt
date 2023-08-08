package org.duckietown.hatchery.achdjian

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.MessageType
import com.intellij.openapi.wm.*

fun showMessage(project: Project, messageType: MessageType, message: String) {
    ApplicationManager.getApplication().invokeLater {
        val toolWindowManager = ToolWindowManager.getInstance(project)
        if (toolWindowManager.canShowNotification(ToolWindowId.RUN)) {
            toolWindowManager.notifyByBalloon(ToolWindowId.MESSAGES_WINDOW, messageType, message, null, null)
        }
    }
}