package it.achdjian.plugin.ros.importer

import com.intellij.icons.AllIcons
import com.intellij.ide.impl.ProjectUtil
import com.intellij.ide.util.PropertiesComponent
import com.intellij.openapi.actionSystem.AnAction
import com.intellij.openapi.actionSystem.AnActionEvent
import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.fileChooser.FileChooserFactory
import com.intellij.openapi.project.DumbAware
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.Messages
import com.intellij.openapi.util.IconLoader
import com.intellij.openapi.vfs.LocalFileSystem
import com.intellij.openapi.vfs.VirtualFile
import com.jetbrains.cidr.cpp.cmake.projectWizard.CLionProjectWizardUtils
import com.jetbrains.cidr.cpp.cmake.workspace.CMakeWorkspace
import it.achdjian.plugin.ros.utils.getRosVersionFromCMakeLists
import it.achdjian.plugin.ros.utils.releaseProfile
import java.io.File
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths
import javax.swing.Icon

class ImporterRosWorkspaceAction : AnAction(AllIcons.ToolbarDecorator.Import), DumbAware {
  override fun actionPerformed(event: AnActionEvent) {
    startWizard()
  }
}

class MyFileChooserDescriptor : FileChooserDescriptor(false, true, false, false, false, false) {
  override fun getIcon(file: VirtualFile): Icon? =
    if (isRosWS(file)) IconLoader.getIcon("/icons/rosFolder.svg") else super.getIcon(file)

  override fun isFileSelectable(file: VirtualFile?) =
    file != null && (!file.isDirectory || file.children.isNotEmpty())
}

fun isRosWS(file: VirtualFile): Boolean {
  if (file.isDirectory) {
    val src = file.findChild("src")
    src?.let {
      val cmakeLists = it.findChild("CMakeLists.txt")
      if (cmakeLists != null && cmakeLists.exists())
        return true
    }
  }
  return false
}

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

fun undefinedRosEnvironment() {
  Messages.showErrorDialog("Directory doesn't contains a recognized ROS environment", "Import ROS workspace");
}

fun invalidRosEnvironment() {
  Messages.showErrorDialog("Directory doesn't contains a valid ROS environment", "Import ROS workspace");
}

fun choseFile(): VirtualFile? {
  val fileChooserDescriptor = MyFileChooserDescriptor()
  fileChooserDescriptor.isHideIgnored = true
  fileChooserDescriptor.title = "Select Directory to import"
  val lastImportedLocation = PropertiesComponent.getInstance().getValue("last.imported.location")
  val files = lastImportedLocation?.let {
    LocalFileSystem.getInstance().refreshAndFindFileByPath(lastImportedLocation)
  }?.let { arrayOf(it) } ?: arrayOf()
  val fileChooser = FileChooserFactory.getInstance().createFileChooser(fileChooserDescriptor, null, null)
  val filesChose = fileChooser.choose(null, *files)
  return if (filesChose.isEmpty()) {
    null
  } else {
    PropertiesComponent.getInstance().setValue("last.imported.location", filesChose[0].path)
    filesChose[0]
  }
}

