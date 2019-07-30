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

fun getResourceAsString(resourceName: String) =
  RosEnvironments::class.java.classLoader.getResourceAsStream(resourceName)
    ?.let { FileUtil.loadTextAndClose(it) } ?: ""

fun createMainCMakeLists() = getResourceAsString("templates/CMakeLists.txt")

fun releaseProfile(version: RosVersion, baseDir: File): CMakeSettings.Profile =
  CMakeSettings.Profile(
    name = "Release",
    buildType = "Release",
    toolchainName = "",
    generationOptions = "-DCATKIN_DEVEL_PREFIX=$baseDir/devel -DCMAKE_INSTALL_PREFIX=$baseDir/install",
    passSystemEnvironment = true,
    additionalEnvironment = version.env,
    generationDir = File(baseDir, "build"),
    buildOptions = "")

fun getRosVersionFromCMakeLists(file: VirtualFile): RosVersionImpl? =
  getCMakeListsTarget(file)?.let { getRosEnvironment().getOwnerVersion(it) }

fun getCMakeListsTarget(file: VirtualFile): Path? =
  Paths.get(file.path).let { if (Files.isSymbolicLink(it)) Files.readSymbolicLink(it) else null }