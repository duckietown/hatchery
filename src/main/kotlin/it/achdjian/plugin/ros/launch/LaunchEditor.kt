package it.achdjian.plugin.ros.launch

import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.ComboBox
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.ui.components.JBTextField
import com.intellij.ui.components.fields.IntegerField
import com.intellij.ui.layout.panel
import it.achdjian.plugin.ros.ui.LauncherFileChooser
import java.io.File
import javax.swing.JCheckBox


object LaunchFileChooserDescriptor : FileChooserDescriptor(true, false, false, false, false, false)

class LaunchEditor( project: Project) : SettingsEditor<LaunchConfiguration>() {
    private val browseButton = LauncherFileChooser("Launch file", project, LaunchFileChooserDescriptor)
    private val rosMasterAddr = JBTextField("127.0.0.1")
    private val rosMasterPort = IntegerField("11311", 0, 65535)
    private val screen = JCheckBox("Force output of all local nodes to screen")
//    private val log = JCheckBox("Force output of all local nodes to log")
    private val wait = JCheckBox("wait for master to start before launching")
    private val verbose = JCheckBox("verbose printing")
//    private val loggerLevel = ComboBox<String>()

    init {
//        loggerLevel.addItem("debug")
//        loggerLevel.addItem("info")
//        loggerLevel.addItem("warn")
//        loggerLevel.addItem("error")
//        loggerLevel.addItem("fatal")
//        loggerLevel.selectedItem = "info"

        rosMasterPort.value = 11311
    }

    override fun applyEditorTo(launchConfiguration: LaunchConfiguration) {
        launchConfiguration.path = VfsUtil.findFileByIoFile(File(browseButton.text), true)
        launchConfiguration.rosMasterAddr=rosMasterAddr.text
        launchConfiguration.rosMasterPort=rosMasterPort.value
        launchConfiguration.screen=screen.isSelected
//        launchConfiguration.log=log.isSelected
        launchConfiguration.wait=wait.isSelected
        launchConfiguration.verbose=verbose.isSelected
//        launchConfiguration.logLevel=loggerLevel.selectedItem as String
    }

    override fun resetEditorFrom(launchConfiguration: LaunchConfiguration) {
        launchConfiguration.path?.let { path ->
            browseButton.text = path.path
        }
        rosMasterAddr.text = launchConfiguration.rosMasterAddr
        rosMasterPort.value = launchConfiguration.rosMasterPort
        screen.isSelected = launchConfiguration.screen
//        log.isSelected = launchConfiguration.log
        wait.isSelected = launchConfiguration.wait
        verbose.isSelected = launchConfiguration.verbose
//        loggerLevel.selectedItem = launchConfiguration.logLevel
    }

    override fun createEditor() = panel {
        row("Launch file") {
            browseButton(grow)
        }
        titledRow("ROS MASTER") {
            row("ROS MASTER address") {
                rosMasterAddr(grow)
            }
            row("ROS MASTER port") {
                rosMasterPort(grow)
            }
        }
        titledRow("Options") {
//            row("logger level") {
//                loggerLevel(grow)
//            }
            row {
                screen(grow)
            }
//            row {
//                log(grow)
//            }
            row {
                wait(grow)
            }
            row {
                verbose(grow)
            }
        }

    }
}