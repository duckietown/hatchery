package edu.umontreal.hatchery.rospackage

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

/*
 * http://wiki.ros.org/Manifest
 */

object RosPackageFileType : LanguageFileType(XMLLanguage.INSTANCE) {
  const val filename = "package.xml"

  override fun getName() = filename

  override fun getDescription() = "rospackage"

  override fun getDefaultExtension() = "xml"

  override fun getIcon() = Icons.package_file
}