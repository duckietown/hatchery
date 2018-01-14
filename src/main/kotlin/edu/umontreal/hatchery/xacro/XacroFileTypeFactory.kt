package edu.umontreal.hatchery.xacro

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class XacroFileTypeFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(XacroFileType, "xacro")

  object XacroFileType : LanguageFileType(XMLLanguage.INSTANCE) {
    override fun getName() = "xacro_file_name"
    override fun getDescription() = "xacro_file_description"
    override fun getDefaultExtension() = "xacro"
    override fun getIcon() = Icons.ros_file
  }
}