package edu.umontreal.hatchery

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType

class ROSPackageFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(FileType, "xml")

    object ROSPackageLang : XMLLanguage("ROSPackageLang", "application/xml", "text/xml")

    object FileType : LanguageFileType(ROSPackageLang) {
        override fun getName() = "package manifest file"
        override fun getDescription() = "Manifest for ROS packages"
        override fun getDefaultExtension() = "xml"
        override fun getIcon() = Icons.package_file
    }
}