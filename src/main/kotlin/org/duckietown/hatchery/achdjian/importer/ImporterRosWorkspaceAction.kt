package org.duckietown.hatchery.achdjian.importer

import com.intellij.icons.AllIcons
import com.intellij.ide.impl.ProjectUtil
import com.intellij.ide.util.PropertiesComponent
import com.intellij.openapi.actionSystem.*
import com.intellij.openapi.fileChooser.*
import com.intellij.openapi.project.*
import com.intellij.openapi.ui.Messages
import com.intellij.openapi.util.IconLoader
import com.intellij.openapi.vfs.*
import com.jetbrains.cidr.cpp.cmake.projectWizard.CLionProjectWizardUtils
import com.jetbrains.cidr.cpp.cmake.workspace.CMakeWorkspace
import org.duckietown.hatchery.achdjian.utils.*
import org.duckietown.hatchery.filesystem.Icons
import java.io.File
import javax.swing.Icon

class ImporterRosWorkspaceAction : AnAction(AllIcons.ToolbarDecorator.Import), DumbAware {
    override fun actionPerformed(event: AnActionEvent) = startWizard()
}

class RosWorkspaceChooserDescriptor :
        FileChooserDescriptor(false, true, false, false, false, false) {
    override fun getIcon(file: VirtualFile): Icon? =
            if (isRosWS(file)) Icons.ros_folder else super.getIcon(file)

    override fun isFileSelectable(file: VirtualFile?) =
            file != null && (!file.isDirectory || file.children.isNotEmpty())
}

fun isRosWS(file: VirtualFile) =
        file.isDirectory &&
                file.findChild("src")
                        ?.findChild("CMakeLists.txt")
                        ?.exists() ?: false

fun startWizard() {
    choseFile()?.let { chosenDir ->
        CLionProjectWizardUtils.refreshProjectDir(chosenDir)
        if (isRosWS(chosenDir)) {
            chosenDir.findChild("src")?.findChild("CMakeLists.txt")?.let { cMakeList ->
                val rosVersion = getRosVersionFromCMakeLists(cMakeList)
                rosVersion?.let { version ->
                    CMakeWorkspace.forceReloadOnOpening(cMakeList)
                    val project = ProjectUtil.openOrImport(cMakeList.path, null as Project?, false)
                    project?.let {
                        val cMakeWorkspace = CMakeWorkspace.getInstance(it)
                        val settings = cMakeWorkspace.settings
                        val releaseProfile = releaseProfile(version, File(chosenDir.path))
                        settings.profiles = listOf(releaseProfile)
                    }
                } ?: undefinedRosEnvironment()
            } ?: invalidRosEnvironment()
        }
    }
}

fun undefinedRosEnvironment() =
        Messages.showErrorDialog("Directory doesn't contains a recognized ROS environment", "Import ROS workspace")

fun invalidRosEnvironment() =
        Messages.showErrorDialog("Directory doesn't contains a valid ROS environment", "Import ROS workspace")

fun choseFile(): VirtualFile? {
    val fileChooserDescriptor = RosWorkspaceChooserDescriptor()
    fileChooserDescriptor.isHideIgnored = true
    fileChooserDescriptor.title = "Select Directory to import"
    val lastImportedLocation = PropertiesComponent.getInstance().getValue("last.imported.location")
    val files = lastImportedLocation?.let {
        LocalFileSystem.getInstance().refreshAndFindFileByPath(lastImportedLocation)
    }?.let { arrayOf(it) } ?: arrayOf()
    val fileChooser = FileChooserFactory.getInstance().createFileChooser(fileChooserDescriptor, null, null)
    val filesChosen = fileChooser.choose(null, *files)
    return if (filesChosen.isEmpty()) null
    else filesChosen[0].apply { PropertiesComponent.getInstance().setValue("last.imported.location", path) }
}
