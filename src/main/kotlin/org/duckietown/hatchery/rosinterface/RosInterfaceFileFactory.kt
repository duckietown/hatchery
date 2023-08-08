package org.duckietown.hatchery.rosinterface

import com.intellij.openapi.fileTypes.*

class RosInterfaceFileFactory : FileTypeFactory() {
  val ROSINTERFACE_EXTENSIONS = "msg;srv"

  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosInterfaceFileType, ROSINTERFACE_EXTENSIONS)
}