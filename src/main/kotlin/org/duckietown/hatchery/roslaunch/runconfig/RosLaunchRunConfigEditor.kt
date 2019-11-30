package org.duckietown.hatchery.roslaunch.runconfig

import com.intellij.openapi.fileChooser.FileChooserDescriptor
import com.intellij.openapi.options.SettingsEditor
import com.intellij.openapi.project.Project
import com.intellij.openapi.vfs.VfsUtil
import com.intellij.ui.components.JBTextField
import com.intellij.ui.components.fields.IntegerField
import com.intellij.ui.layout.panel
import org.duckietown.hatchery.achdjian.ui.LauncherFileChooser
import java.io.File
import javax.swing.JCheckBox

class RosLaunchRunConfigEditor(project: Project) : SettingsEditor<RosLaunchRunConfiguration>() {
  private val fileDescriptor = FileChooserDescriptor(true, false, false, false, false, false)
  private val browseButton = LauncherFileChooser("Launch file", project, fileDescriptor)
  private val rosMasterAddr = JBTextField("127.0.0.1")
  private val rosMasterPort = IntegerField("11311", 0, 65535)
  private val screen = JCheckBox("Force output of all local nodes to screen")
//private val log = JCheckBox("Force output of all local nodes to log")
  private val wait = JCheckBox("wait for master to start before launching")
  private val verbose = JCheckBox("verbose printing")
//private val loggerLevel = ComboBox<String>()

  init {
//  loggerLevel.addItem("debug")
//  loggerLevel.addItem("info")
//  loggerLevel.addItem("warn")
//  loggerLevel.addItem("error")
//  loggerLevel.addItem("fatal")
//  loggerLevel.selectedItem = "info"

    rosMasterPort.value = 11311
  }

  override fun applyEditorTo(rosLaunchRunConfiguration: RosLaunchRunConfiguration) {
    rosLaunchRunConfiguration.path = VfsUtil.findFileByIoFile(File(browseButton.text), true)
    rosLaunchRunConfiguration.rosMasterAddr = rosMasterAddr.text
    rosLaunchRunConfiguration.rosMasterPort = rosMasterPort.value
    rosLaunchRunConfiguration.screen = screen.isSelected
//  launchConfiguration.log=log.isSelected
    rosLaunchRunConfiguration.wait = wait.isSelected
    rosLaunchRunConfiguration.verbose = verbose.isSelected
//  launchConfiguration.logLevel=loggerLevel.selectedItem as String
  }

  override fun resetEditorFrom(rosLaunchRunConfiguration: RosLaunchRunConfiguration) {
    rosLaunchRunConfiguration.path?.let { path -> browseButton.text = path.path }
    rosMasterAddr.text = rosLaunchRunConfiguration.rosMasterAddr
    rosMasterPort.value = rosLaunchRunConfiguration.rosMasterPort
    screen.isSelected = rosLaunchRunConfiguration.screen
//  log.isSelected = launchConfiguration.log
    wait.isSelected = rosLaunchRunConfiguration.wait
    verbose.isSelected = rosLaunchRunConfiguration.verbose
//  loggerLevel.selectedItem = launchConfiguration.logLevel
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
//    row("logger level") {
//      loggerLevel(grow)
//    }
      row {
        screen(grow)
      }
//    row {
//      log(grow)
//    }
      row {
        wait(grow)
      }
      row {
        verbose(grow)
      }
    }
  }
}