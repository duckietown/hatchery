package it.achdjian.plugin.ros.utils

import com.intellij.openapi.diagnostic.Logger
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.file.FileSystems
import java.nio.file.Path

fun catkinFindLibexec(packageName: String, env: Map<String, String>): List<Path> {
  val log = Logger.getInstance("#it.achdjian.plugin.ros.utils.Catkin.catkinFindLibexec")
  val process =
    ProcessBuilder()
      .command("catkin_find", "--first-only", "--without-underlays", "--libexec", packageName)
      .run {
        environment().putAll(env)
        start()
      }

  log.trace("find libexec for $packageName")
  return BufferedReader(InputStreamReader(process.inputStream)).lineSequence()
    .map {
      log.trace(it)
      FileSystems.getDefault().getPath(it)
    }.toList()
}