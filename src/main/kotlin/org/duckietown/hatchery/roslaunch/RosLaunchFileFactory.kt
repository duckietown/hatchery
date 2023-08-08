package org.duckietown.hatchery.roslaunch

import com.intellij.openapi.fileTypes.*

class RosLaunchFileFactory : FileTypeFactory() {
  private val ROSLAUNCH_EXTENSIONS = "launch;test"

  override fun createFileTypes(consumer: FileTypeConsumer) =
    consumer.consume(RosLaunchFileType, ROSLAUNCH_EXTENSIONS)
}