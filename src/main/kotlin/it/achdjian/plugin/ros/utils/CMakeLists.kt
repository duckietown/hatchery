package it.achdjian.plugin.ros.utils

import com.intellij.openapi.util.io.FileUtil
import com.intellij.openapi.vfs.VirtualFile
import com.jetbrains.cidr.cpp.cmake.CMakeSettings
import it.achdjian.plugin.ros.data.RosEnvironments
import it.achdjian.plugin.ros.data.RosVersion
import it.achdjian.plugin.ros.data.RosVersionImpl
import it.achdjian.plugin.ros.data.getRosEnvironment
import java.io.File
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths

fun getResourceAsString(resourceName: String): String {
  val resource = RosEnvironments::class.java.classLoader.getResourceAsStream(resourceName)
  return resource?.let { FileUtil.loadTextAndClose(resource) } ?: ""
}

fun createMainCMakeLists() = getResourceAsString("templates/CMakeLists.txt")

fun releaseProfile(version: RosVersion, baseDir: File): CMakeSettings.Profile {
  val buildDir = File(baseDir, "build")
  val options = "-DCATKIN_DEVEL_PREFIX=${baseDir}/devel -DCMAKE_INSTALL_PREFIX=${baseDir}/install"
  return CMakeSettings.Profile(
    "Release",
    "Release",
    "",
    options,
    true,
    version.env,
    buildDir,
    "")
}

fun getRosVersionFromCMakeLists(file: VirtualFile): RosVersionImpl? =
  getCMakeListsTarget(file)?.let { getRosEnvironment().getOwnerVersion(it) }

fun getCMakeListsTarget(file: VirtualFile): Path? =
  Paths.get(file.path).let { if (Files.isSymbolicLink(it)) Files.readSymbolicLink(it) else null }