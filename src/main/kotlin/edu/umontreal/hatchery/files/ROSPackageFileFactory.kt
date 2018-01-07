package edu.umontreal.hatchery.files

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.ExactFileNameMatcher
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.Icons

class ROSPackageFileFactory : FileTypeFactory() {
    override fun createFileTypes(ftc: FileTypeConsumer) = ftc.consume(FileType, ExactFileNameMatcher("package.xml"))

    object FileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "package manifest file"
        override fun getDescription() = "Manifest for ROS packages"
        override fun getDefaultExtension() = "xml"
        override fun getIcon() = Icons.package_file
    }
}