package edu.umontreal.hatchery.rviz

import org.jetbrains.yaml.YAMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory


import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class RVizFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(FileType, "rviz")

    object FileType : LanguageFileType(YAMLLanguage.INSTANCE) {
        override fun getName() = "rviz"
        override fun getDescription() = "rviz"
        override fun getDefaultExtension() = "rviz"
        override fun getIcon() = Icons.ros_file
    }
}