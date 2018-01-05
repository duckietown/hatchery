package edu.umontreal.hatchery.filesystem

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType

class ROSLaunchFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosLaunchFileType, "launch;test")

    object RosLaunchFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "roslaunch"
        override fun getDescription() = "roslaunch"
        override fun getDefaultExtension() = "launch"
        override fun getIcon() = Icons.launch_file
    }
}