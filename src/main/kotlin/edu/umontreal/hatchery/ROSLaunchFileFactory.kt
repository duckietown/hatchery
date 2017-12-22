package edu.umontreal.hatchery

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType

class ROSLaunchFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosLaunchFileType, "launch")

    object RosLaunchFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "roslaunch file"
        override fun getDescription() = "roslaunch file"
        override fun getDefaultExtension() = "launch"
        override fun getIcon() = Icons.file
    }
}