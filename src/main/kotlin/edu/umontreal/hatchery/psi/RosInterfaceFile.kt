package edu.umontreal.hatchery.psi

import com.intellij.extapi.psi.PsiFileBase
import com.intellij.psi.FileViewProvider
import edu.umontreal.hatchery.filesystem.Icons
import edu.umontreal.hatchery.rosinterface.RosInterfaceFileType
import edu.umontreal.hatchery.rosinterface.RosInterfaceLanguage

/*
 * https://github.com/ros2/ros2/wiki/About-ROS-Interfaces
 */

class RosInterfaceFile(viewProvider: FileViewProvider) : PsiFileBase(viewProvider, RosInterfaceLanguage) {
  override fun getFileType() = RosInterfaceFileType

  override fun toString() = "ROS Interface File"

  override fun getIcon(flags: Int) = Icons.robot
}