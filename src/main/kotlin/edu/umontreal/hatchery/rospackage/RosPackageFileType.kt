package edu.umontreal.hatchery.rospackage

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

object RosPackageFileType : LanguageFileType(XMLLanguage.INSTANCE) {
  override fun getName() = "rospackage"

  override fun getDescription() = "rospackage"

  override fun getDefaultExtension() = "xml"

  override fun getIcon() = Icons.package_file
}