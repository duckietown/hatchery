package edu.umontreal.hatchery.xacro

import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

object XacroFileTypeFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(XacroFileType, "xacro")
}