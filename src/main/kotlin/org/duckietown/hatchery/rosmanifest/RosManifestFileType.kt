package org.duckietown.hatchery.rosmanifest

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.LanguageFileType
import org.duckietown.hatchery.filesystem.Icons

/*
 * http://wiki.ros.org/Manifest
 */

object RosManifestFileType : LanguageFileType(XMLLanguage.INSTANCE) {
  const val filename = "package.xml"

  override fun getName() = filename

  override fun getDescription() = "rosmanifest"

  override fun getDefaultExtension() = "xml"

  override fun getIcon() = Icons.package_file
}