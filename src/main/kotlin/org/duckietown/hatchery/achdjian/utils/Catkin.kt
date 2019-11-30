package org.duckietown.hatchery.achdjian.utils

import com.intellij.openapi.diagnostic.Logger
import java.io.*
import java.nio.file.*

fun catkinFindLibexec(packageName: String, env: Map<String, String>): List<Path> {
    val log = Logger.getInstance("#org.duckietown.hatchery.achdjian.utils.Catkin.catkinFindLibexec")
    val processBuilder = ProcessBuilder().command("catkin_find", "--first-only", "--without-underlays", "--libexec", packageName)
    processBuilder.environment().putAll(env)
    val process = processBuilder.start()

    val pathList = ArrayList<Path>()
    val reader = BufferedReader(InputStreamReader(process.inputStream))
    log.trace("find libexec for $packageName")
    while (true) {
        val line = reader.readLine() ?: break
        log.trace(line)
        pathList.add(FileSystems.getDefault().getPath(line))
    }

    return pathList
}