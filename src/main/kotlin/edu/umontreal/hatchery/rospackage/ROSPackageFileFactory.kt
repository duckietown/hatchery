package edu.umontreal.hatchery.rospackage

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.ExactFileNameMatcher
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class ROSPackageFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(ROSPackageFileType, ExactFileNameMatcher("package.xml"))

    object ROSPackageFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "rospackage"

        override fun getDescription() = "rospackage"

        override fun getDefaultExtension() = "xml"

        override fun getIcon() = Icons.package_file
    }
}