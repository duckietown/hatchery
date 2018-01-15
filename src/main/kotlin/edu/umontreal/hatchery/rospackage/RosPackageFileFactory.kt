package edu.umontreal.hatchery.rospackage

import com.intellij.openapi.fileTypes.*

class RosPackageFileFactory : FileTypeFactory() {
  private object PackageFileNameMatcher : ExactFileNameMatcher(RosPackageFileType.filename)

  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosPackageFileType, PackageFileNameMatcher)
}