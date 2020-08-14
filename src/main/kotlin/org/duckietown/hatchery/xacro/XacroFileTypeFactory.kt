package org.duckietown.hatchery.xacro

import com.intellij.openapi.fileTypes.*

object XacroFileTypeFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(XacroFileType, "xacro")
}