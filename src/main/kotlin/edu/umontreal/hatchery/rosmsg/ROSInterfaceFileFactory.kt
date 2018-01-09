package edu.umontreal.hatchery.rosmsg

import com.intellij.lang.xml.XMLLanguage
import com.intellij.openapi.fileTypes.ExactFileNameMatcher
import com.intellij.openapi.fileTypes.FileTypeConsumer
import com.intellij.openapi.fileTypes.FileTypeFactory
import com.intellij.openapi.fileTypes.LanguageFileType
import edu.umontreal.hatchery.filesystem.Icons

class ROSInterfaceFileFactory : FileTypeFactory() {
    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(ROSInterfaceFileType, "msg;srv")

    object ROSInterfaceFileType : LanguageFileType(XMLLanguage.INSTANCE) {
        override fun getName() = "rospackage"
        override fun getDescription() = "rospackage"
        override fun getDefaultExtension() = "xml"
        override fun getIcon() = Icons.package_file
    }
}