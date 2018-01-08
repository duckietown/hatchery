package edu.umontreal.hatchery.catkin

import com.intellij.ide.IconProvider
import com.intellij.psi.PsiDirectory
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFile
import edu.umontreal.hatchery.filesystem.Icons

class CatkinIconProvider : IconProvider() {
    // TODO: Figure out why element is never PsiDirectory
    override fun getIcon(element: PsiElement, flags: Int) =
            if (element is PsiDirectory && hasPackageXml(element)) Icons.catkin_file else null

    private fun hasPackageXml(element: PsiDirectory) = element.files.any { it.name == "package.xml" }

//    override fun getIcon(file: VirtualFile, f: Int, p: Project?) = if (hasPackageXml(file)) Icons.catkin_file else null
//
//    private fun hasPackageXml(file: VirtualFile) = file.isDirectory && file.children.any { it.name == "package.xml" }

//    override fun getIcon(file: VirtualFile, flags: Int, project: Project?): Icon? {
//        if (project == null) return null
//        val files = FilenameIndex.getFilesByName(project, "package.xml", GlobalSearchScope.allScope(project))
//        return if(files.any { it.parent?.virtualFile === file}) Icons.catkin_file else null
//    }

//    override fun createFileTypes(consumer: FileTypeConsumer) = consumer.consume(RosLaunchFileType, "launch;test")
//
//    object RosLaunchFileType : LanguageFileType(XMLLanguage.INSTANCE) {
//        override fun getName() = "roslaunch"
//        override fun getDescription() = "roslaunch"
//        override fun getDefaultExtension() = "launch"
//        override fun getIcon() = Icons.launch_file
//    }
}
