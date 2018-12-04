package edu.umontreal.hatchery.ros

import edu.umontreal.hatchery.ros.nodes.Listener
import edu.umontreal.hatchery.ros.nodes.Talker
import org.ros.RosCore
import org.ros.node.DefaultNodeMainExecutor
import org.ros.node.NodeConfiguration
import java.net.Inet4Address
import java.net.UnknownHostException
import java.util.concurrent.TimeUnit

private var mRosCore: RosCore? = null
fun main(args: Array<String>) {

  System.setProperty("javax.net.debug", "all")

  for (s in args) {
    println(s)
  }

  //String IP = "localhost";
  var IP = "192.168.1.244"
  if (args.isNotEmpty()) {
    IP = args[0]
  }

  mRosCore = RosCore.newPublic(IP, 11311)
  mRosCore!!.start()
  try {
    mRosCore!!.awaitStart(5, TimeUnit.SECONDS)
  } catch (e: InterruptedException) {
    e.printStackTrace()
  }

  println("Ros core started")

  startChatter()

  //TODO Improve this way
  while (true) {
  }
}

@Throws(UnknownHostException::class)
private fun startChatter() {

  val e = DefaultNodeMainExecutor.newDefault()

  println("Starting listener node...")
  val listenerConfig = NodeConfiguration.newPublic(Inet4Address.getLocalHost().hostAddress)
  listenerConfig.masterUri = mRosCore?.uri
  listenerConfig.setNodeName("Listener")
  val listener = Listener()
  e.execute(listener, listenerConfig)


  println("Starting talker node...")
  //NodeConfiguration talkerConfig = NodeConfiguration.newPublic(Inet4Address.getLocalHost().getHostAddress());
  val talkerConfig = NodeConfiguration.newPublic("ev3dev")
  talkerConfig.masterUri = mRosCore!!.uri
  talkerConfig.setNodeName("Talker")
  val talker = Talker()
  e.execute(talker, talkerConfig)


}
