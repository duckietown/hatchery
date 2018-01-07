package edu.umontreal.hatchery.files

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.Icons

class ROSAcroFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(ROSAcroFileType, "urdf.xacro")

    object ROSAcroFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "rosacro file"
        override fun getDescription() = "rosacro description"
        override fun getDefaultExtension() = "urdf.xacro"
        override fun getIcon() = Icons.ros_file
    }
}