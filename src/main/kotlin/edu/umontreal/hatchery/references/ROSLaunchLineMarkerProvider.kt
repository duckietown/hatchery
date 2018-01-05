package edu.umontreal.hatchery.references

import com.intellij.codeInsight.daemon.RelatedItemLineMarkerInfo
import com.intellij.codeInsight.daemon.RelatedItemLineMarkerProvider
import com.intellij.codeInsight.navigation.NavigationGutterIconBuilder
import com.intellij.openapi.fileTypes.FileTypeManager
import com.intellij.openapi.project.Project
import com.intellij.openapi.vfs.VirtualFile
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiManager
import com.intellij.psi.search.FileTypeIndex
import com.intellij.psi.search.GlobalSearchScope
import com.intellij.psi.xml.XmlAttributeValue
import com.intellij.util.indexing.FileBasedIndex
import edu.umontreal.hatchery.filesystem.Icons
import java.util.*


class ROSLaunchLineMarkerProvider : RelatedItemLineMarkerProvider() {
    override fun collectNavigationMarkers(element: PsiElement, result: MutableCollection<in RelatedItemLineMarkerInfo<PsiElement>>) {
        val project = element.project
        val manager = PsiManager.getInstance(project)
        if (isRosLaunchFileSubstitution(element)) {
            val relPath = (element as XmlAttributeValue).value!!.substringAfter("$(find ").replace(")", "")
            if (!relPath.contains("(")) {
                val builder = NavigationGutterIconBuilder.create(Icons.resource_file)
                val targets = findFileByRelativePath(project, relPath).map {
                    manager.findFile(it) ?: manager.findDirectory(it)
                }
                if (!targets.isEmpty()) {
                    builder.setTooltipText(targets.first()?.virtualFile?.path!!)
                    builder.setTargets(targets)
                    result.add(builder.createLineMarkerInfo(element))
                } else {
                    result.add(NavigationGutterIconBuilder.create(Icons.broken_resource)
                            .setTooltipText("Broken resource!")
                            .setTarget(element).createLineMarkerInfo(element))
                }
            }
        }
    }

    companion object {
        fun isRosLaunchFileSubstitution(element: PsiElement): Boolean {
            if (element is XmlAttributeValue && element.value!!.startsWith("$(find ")) {
                return true
            }
            return false
        }
    }

    fun findFileByRelativePath(project: Project, fileRelativePath: String): List<VirtualFile> {
        val relativePath = if (fileRelativePath.startsWith("/")) fileRelativePath else "/" + fileRelativePath
        val fileTypes = Collections.singleton(FileTypeManager.getInstance().getFileTypeByFileName(relativePath))
        val fileList = ArrayList<VirtualFile>()
        FileBasedIndex.getInstance().processFilesContainingAllKeys(FileTypeIndex.NAME, fileTypes, GlobalSearchScope.projectScope(project), null)
                { virtualFile ->
                    if (virtualFile.path.endsWith(relativePath)) {
                        fileList.add(virtualFile)
                    }
                    true
                }

        return fileList
    }
}