package edu.umontreal.hatchery.rosinterface

import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

object RosInterfaceFileFactory : FileTypeFactory() {
  const val ROSINTERFACE_EXTENSIONS = "msg;srv"

  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosInterfaceFileType, ROSINTERFACE_EXTENSIONS)
}