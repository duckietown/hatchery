package edu.umontreal.hatchery.rosinterface

import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

object RosInterfaceFileType : LanguageFileType(RosInterfaceLanguage) {
  override fun getName() = "ROS Interface"
  override fun getDescription() = "ROS Interface"
  override fun getDefaultExtension() = "msg"
  override fun getIcon() = Icons.ros_msg
}