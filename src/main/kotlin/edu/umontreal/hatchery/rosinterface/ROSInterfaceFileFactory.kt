package edu.umontreal.hatchery.rosinterface

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class ROSInterfaceFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(ROSInterfaceFileType, "msg;srv")

    object ROSInterfaceFileType : LanguageFileType(ROSInterfaceLanguage) {
        override fun getName() = "ROS Interface"
        override fun getDescription() = "ROS Interface"
        override fun getDefaultExtension() = "msg"
        override fun getIcon() = Icons.ros_msg
    }
}