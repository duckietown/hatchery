package edu.umontreal.hatchery

import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

class ROSLaunchFileTypeFactory: FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(FileType, "launch")
}