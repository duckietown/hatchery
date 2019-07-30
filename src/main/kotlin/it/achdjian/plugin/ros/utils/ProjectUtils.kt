package it.achdjian.plugin.ros.utils

import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.project.Project
import it.achdjian.plugin.ros.data.RosPackage
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.file.FileSystems
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.attribute.PosixFilePermission
import java.nio.file.attribute.PosixFilePermissions
import java.util.*

object CONST {
  val perms: EnumSet<PosixFilePermission> = EnumSet.of<PosixFilePermission>(PosixFilePermission.OWNER_READ, PosixFilePermission.OWNER_WRITE, PosixFilePermission.OWNER_EXECUTE, PosixFilePermission.GROUP_READ)
}

/**
 * TODO: Integrate with [edu.umontreal.hatchery.ros.Distro.version]
 */

fun getVersion(project: Project) =
  project.projectFile?.parent?.parent
    ?.findChild("CMakeLists.txt")
    ?.let { getRosVersionFromCMakeLists(it) }

fun getBaseDir(project: Project) = project.projectFile?.parent?.parent?.parent

/**
 * TODO: Integrate with [edu.umontreal.hatchery.ros.Ros.packages]
 */

fun getPackages(project: Project): List<RosPackage> {
  val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.getPackages.${project.name}")
  val packages = ArrayList<RosPackage>()
  getVersion(project)?.let { version ->
    val path = FileSystems.getDefault().getPath(project.projectFilePath)
    log.trace("$path")
    val baseSearch = path.parent.parent
    Files.walk(baseSearch)
      .filter { it.fileName.toString() == "package.xml" }
      .peek { log.trace("found package $it") }
      .map { RosPackage(it.parent, mapOf()) }
      .forEach { packages.add(it) }
    packages.addAll(version.searchPackages())
  }
  return packages
}