package org.duckietown.hatchery.achdjian.ui

import com.intellij.openapi.project.Project
import com.intellij.ui.table.JBTable
import com.intellij.webcore.packaging.*

class RosInstalledPackagesPanel(val project: Project, val area: PackagesNotificationPanel) : InstalledPackagesPanel(project, area) {

    val packagesTable = JBTable(RosTablePackageModel())

    protected override fun installEnabled() = false
}