package it.achdjian.plugin.ros.ui

import com.intellij.openapi.project.Project
import com.intellij.ui.table.JBTable
import com.intellij.webcore.packaging.InstalledPackagesPanel
import com.intellij.webcore.packaging.PackagesNotificationPanel

class RosInstalledPackagesPanel(val project: Project, val area: PackagesNotificationPanel) : InstalledPackagesPanel(project, area){

    val packagesTable = JBTable(RosTablePackageModel())

    protected override fun installEnabled() = false
}