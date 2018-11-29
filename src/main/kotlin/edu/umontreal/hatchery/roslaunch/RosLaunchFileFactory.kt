package edu.umontreal.hatchery.roslaunch

import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

object RosLaunchFileFactory : FileTypeFactory() {
  private const val ROSLAUNCH_EXTENSIONS = "launch;test"

  override fun createFileTypes(consumer: FileTypeConsumer) =
    consumer.consume(RosLaunchFileType, ROSLAUNCH_EXTENSIONS)
}