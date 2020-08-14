package org.duckietown.hatchery.ros

import org.duckietown.hatchery.ros.nodes.*
import org.ros.RosCore
import org.ros.node.*
import java.net.Inet4Address
import java.util.concurrent.TimeUnit.SECONDS

fun main() {
  System.setProperty("javax.net.debug", "all")
  val hostAddress = Inet4Address.getLocalHost().hostAddress
  println("Host address: $hostAddress")

  val rosCore = RosCore.newPublic("localhost", 11311).apply {
    start()
    awaitStart(5, SECONDS)
  }

  fun makeConfig(name: String) =
    NodeConfiguration.newPublic(hostAddress).apply {
      masterUri = rosCore.uri
      println("Starting $name node...")
      setNodeName(name)
    }

  val nodeExecutor = DefaultNodeMainExecutor.newDefault()
  nodeExecutor.execute(Listener(), makeConfig("Listener"))
  nodeExecutor.execute(Talker(), makeConfig("Talker"))

  while (true) { }
}