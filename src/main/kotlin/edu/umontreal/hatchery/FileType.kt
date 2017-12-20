package edu.umontreal.hatchery

import com.intellij.openapi.fileTypes.LanguageFileType

object FileType : LanguageFileType(ROSLaunchLang) {
    override fun getName() = "roslaunch file"
    override fun getDescription() = "roslaunch file"
    override fun getDefaultExtension() = "launch"
    override fun getIcon() = Icons.file
}