package it.achdjian.plugin.ros.utils

import com.intellij.openapi.diagnostic.Logger
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.file.FileSystems
import java.nio.file.Path

fun catkinFindLibexec(packageName: String, env: Map<String, String>):List<Path>{
    val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.Catkin.catkinFindLibexec")
    val processBuilder = ProcessBuilder().command("catkin_find", "--first-only","--without-underlays","--libexec",packageName)
    processBuilder.environment().putAll(env)
    val process = processBuilder.start()

    val pathList = ArrayList<Path>()
    val reader = BufferedReader(InputStreamReader(process.inputStream))
    log.trace("find libexec for $packageName")
    while(true) {
        val line = reader.readLine() ?: break
        log.trace(line)
        pathList.add(FileSystems.getDefault().getPath(line))
    }

    return pathList
}