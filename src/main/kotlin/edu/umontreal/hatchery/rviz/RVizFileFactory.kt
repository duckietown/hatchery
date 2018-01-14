package edu.umontreal.hatchery.rviz


import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons
import org.jetbrains.yaml.YAMLLanguage

class RVizFileFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RVizFileType, "rviz")

  object RVizFileType : LanguageFileType(YAMLLanguage.INSTANCE) {
    override fun getName() = "rviz_file_name"
    override fun getDescription() = "rviz_file_description"
    override fun getDefaultExtension() = "rviz"
    override fun getIcon() = Icons.ros_file
  }
}