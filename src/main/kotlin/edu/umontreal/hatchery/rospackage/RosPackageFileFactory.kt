package edu.umontreal.hatchery.rospackage

import com.intellij.openapi.fileTypes.ExactFileNameMatcher
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory

class RosPackageFileFactory : FileTypeFactory() {
  private object PackageFileNameMatcher : ExactFileNameMatcher("package.xml")

  override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosPackageFileType, PackageFileNameMatcher)
}