package edu.umontreal.hatchery.roslaunch

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class ROSLaunchFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosLaunchFileType, "launch;test;launch.old")

    object RosLaunchFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "roslaunch"
        override fun getDescription() = "roslaunch"
        override fun getDefaultExtension() = "launch"
        override fun getIcon() = Icons.ros_launch
    }
}