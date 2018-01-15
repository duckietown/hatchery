package edu.umontreal.hatchery.rviz


import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

object RVizFileFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RVizFileType, "rviz")
}