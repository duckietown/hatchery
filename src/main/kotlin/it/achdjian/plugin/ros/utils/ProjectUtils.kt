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


fun getVersion(project: Project) =
        project.projectFile
                ?.parent
                ?.parent
                ?.findChild("CMakeLists.txt")
                ?.let {
                    getRosVersionFromCMakeLists(it)
                }

fun getBaseDir(project: Project) = project.projectFile?.parent?.parent?.parent


fun getPackages(project: Project): List<RosPackage> {
    val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.getPackages.${project.name}")
    val packages = ArrayList<RosPackage>()
    getVersion(project)?.let { version ->
        val path = FileSystems.getDefault().getPath(project.projectFilePath)
        log.trace("$path")
        val baseSearch = path.parent.parent
        Files
                .walk(baseSearch)
                .filter { it.fileName.toString() == "package.xml" }
                .peek { log.trace("found package $it") }
                .map { RosPackage(it.parent, getEnvironmentVariables(project, version.env)) }
                .forEach { packages.add(it) }
        packages.addAll(version.searchPackages())
    }
    return packages
}

fun getEnvironmentVariables(project: Project, env: Map<String, String>): Map<String, String> {
    val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.getEnvironmentVariables.${project.name}")


    val newEnv = HashMap<String, String>()

    val base = FileSystems.getDefault().getPath(project.projectFile?.parent?.parent?.parent?.path)
    base?.let { basePath ->
        basePath
                .resolve("devel")
                ?.resolve("setup.bash")
                ?.let {
                    log.trace("setup.bash at $it")
                    newEnv.putAll(getEnvironment(basePath, it.toString(), env))
                }
    }

    return newEnv
}


fun getEnvironment(basePath: Path, setupFile: String, env: Map<String, String> = emptyMap()): Map<String, String> {
    val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.getEnvironment")
    log.trace("getEnvironment($setupFile)")
    val checkEnvFile = "#!/bin/bash\nsource $setupFile\nenv"
    val tempFile = Files.createTempFile("checkSetup", ".bash", PosixFilePermissions.asFileAttribute(CONST.perms))
    Files.write(tempFile, checkEnvFile.toByteArray())
    val processBuilder = ProcessBuilder().command(tempFile.toString()).directory(basePath.toFile())
    processBuilder.environment().putAll(env)
    val process = processBuilder.start()
    process.waitFor()
    val newEnv = HashMap<String, String>()
    if (process.exitValue() != 0) {
        val errorReader = BufferedReader(InputStreamReader(process.errorStream))
        while (true) {
            val errorLine = errorReader.readLine() ?: break
            log.warn(errorLine)
        }
    } else {
        val reader = BufferedReader(InputStreamReader(process.inputStream))
        log.trace(tempFile.toString())
        while (true) {
            val line = reader.readLine() ?: break
            val assignment = line.indexOfFirst { it == '=' }
            if (assignment > 0) {
                var value = line.substring(assignment + 1)
                val key = line.substring(0, assignment)
                if (value.isNotEmpty()) {
                    if (value[0] == '\"') {
                        value = value.drop(1)
                    }
                    if (value.last() == '\"') {
                        value = value.dropLast(1)
                    }
                }
                newEnv[key] = value
                log.trace("$key=$value")
            } else {
                log.debug("Not parsed: $line")
            }
        }
    }
    return newEnv
}
