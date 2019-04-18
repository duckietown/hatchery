package it.achdjian.plugin.ros.generator

import com.intellij.ide.util.PsiNavigationSupport
import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.module.Module
import com.intellij.openapi.project.Project
import com.intellij.openapi.util.Computable
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.openapi.vfs.VfsUtilCore
import com.intellij.openapi.vfs.VirtualFile
import com.jetbrains.cidr.cpp.cmake.projectWizard.generators.CMakeAbstractCPPProjectGenerator
import com.jetbrains.cidr.cpp.cmake.projectWizard.generators.settings.CMakeProjectSettings
import com.jetbrains.cidr.cpp.cmake.workspace.CMakeWorkspace
import it.achdjian.plagin.ros.ui.panel
import it.achdjian.plugin.ros.data.RosVersion
import it.achdjian.plugin.ros.data.RosVersionNull
import it.achdjian.plugin.ros.data.getRosEnvironment
import it.achdjian.plugin.ros.ui.PackagesPanel
import it.achdjian.plugin.ros.utils.releaseProfile
import java.io.File
import javax.swing.BoxLayout
import javax.swing.JComponent
import javax.swing.JPanel


class RosNodeGenerator : CMakeAbstractCPPProjectGenerator() {

    companion object {
        private val LOG = Logger.getInstance(RosNodeGenerator::class.java)
    }

    private lateinit var version: RosVersion
    private val state = getRosEnvironment()
    private var packagesPanel = PackagesPanel()

    override fun getName(): String = "ROS workspace"

    override fun getSettingsPanel(): JComponent {
        val versionsName = state.versions.map { it.name }

        val panel = JPanel()
        panel.layout = BoxLayout(panel, BoxLayout.Y_AXIS)

        state.versions.firstOrNull()?.let {
            showPackages(it.name)
        }


        val optionPanel = panel("ROS version") {
            row("ROS version") {
                comboBox(versionsName) {
                    showPackages(it.item.toString())
                }
            }
        }

        panel.add(optionPanel)
        panel.add(packagesPanel)

        return panel
    }

    private fun showPackages(versionName: String) {
        version = state.versions.find { version -> version.name == versionName } ?: RosVersionNull
        packagesPanel.setPackages(version.searchPackages())
    }

    override fun createSourceFiles(projectName: String, path: VirtualFile): Array<VirtualFile> {
        createStructure(projectName, path)
        return arrayOf()
    }

    override    fun getCMakeFileContent(p0: String) = ""

    override fun createCMakeFile(name: String, dir: VirtualFile): VirtualFile  = createStructure(name, dir)


    override fun generateProject(project: Project, baseDir: VirtualFile, cmakeSetting: CMakeProjectSettings, module: Module) {
        //super.generateProject(project, path, cmakeSetting, module)
        val cmakeFile = createCMakeFile(project.name, baseDir)
        val srcDir = VfsUtil.createDirectoryIfMissing(baseDir, "src")
        CMakeWorkspace.getInstance(project).selectProjectDir(VfsUtilCore.virtualToIoFile(srcDir))
        if (!ApplicationManager.getApplication().isHeadlessEnvironment) {
            PsiNavigationSupport.getInstance().createNavigatable(project,cmakeFile, -1).navigate(false)
        }
        val cMakeWorkspace = CMakeWorkspace.getInstance(project)
        val settings = cMakeWorkspace.settings
        val releaseProfile = releaseProfile(version, File(baseDir.path))

        settings.profiles = listOf(releaseProfile)
    }

    private fun createStructure( projectName: String, baseDir: VirtualFile) : VirtualFile {
        return ApplicationManager.getApplication().runWriteAction(Computable<VirtualFile> {
            val srcDir = VfsUtil.createDirectoryIfMissing(baseDir, "src")
            LOG.trace("Created src folder: ${srcDir.path}")
            val cmakeLists = srcDir.findChild("CMakeLists.txt")?.let { it } ?: version.initWorkspace(baseDir)
            ?: srcDir.createChildData(this, "CMakeLists.txt")
            version.createPackage(baseDir, projectName, packagesPanel.selected())
            cmakeLists
        })
    }

}