package edu.umontreal.hatchery.filesystem

import com.intellij.lang.Language
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

import com.intellij.openapi.fileTypes.LanguageFileType

class RVizFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(FileType, "rviz")

    object RVizLang : Language("yaml")

    object FileType : LanguageFileType(RVizLang) {
        override fun getName() = "rviz file"
        override fun getDescription() = "rviz description"
        override fun getDefaultExtension() = "rviz"
        override fun getIcon() = Icons.ros_file
    }
}