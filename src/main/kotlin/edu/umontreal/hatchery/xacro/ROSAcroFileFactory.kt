package edu.umontreal.hatchery.xacro

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class ROSAcroFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(AcroFileType, "xacro")

    object AcroFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "xacro"
        override fun getDescription() = "xacro"
        override fun getDefaultExtension() = "xacro"
        override fun getIcon() = Icons.ros_file
    }
}