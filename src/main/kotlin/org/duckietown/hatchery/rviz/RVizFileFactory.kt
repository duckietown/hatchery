package org.duckietown.hatchery.rviz


import com.intellij.openapi.fileTypes.*

class RVizFileFactory : FileTypeFactory() {
  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RVizFileType, "rviz")
}