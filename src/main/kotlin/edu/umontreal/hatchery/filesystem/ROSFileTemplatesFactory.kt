package edu.umontreal.hatchery.filesystem

import com.intellij.ide.fileTemplates.FileTemplateGroupDescriptor
import com.intellij.ide.fileTemplates.FileTemplateGroupDescriptorFactory
import com.intellij.ide.fileTemplates.FileTemplateDescriptor

class ROSFileTemplatesFactory : FileTemplateGroupDescriptorFactory {
    override fun getFileTemplatesDescriptor() =
            FileTemplateGroupDescriptor(HatcheryBundle.message("plugin.description"), Icons.resource_file).apply {
                addTemplate(FileTemplateDescriptor("package.xml", Icons.package_file))
            }
}