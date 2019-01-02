package edu.umontreal.hatchery.ros.nodes


import org.ros.internal.message.Message
import org.ros.message.MessageListener
import org.ros.namespace.GraphName
import org.ros.node.AbstractNodeMain
import org.ros.node.ConnectedNode
import std_msgs.String

class Listener: AbstractNodeMain() {

  override fun getDefaultNodeName() = GraphName.of("rosjava_tutorial/listener")!!

  override fun onStart(connectedNode: ConnectedNode?) {
    val log = connectedNode!!.log
    val subscriber = connectedNode.newSubscriber<std_msgs.String>("chatter", std_msgs.String._TYPE)
    subscriber.addMessageListener { p0 -> System.out.println("I heard: \"" + p0?.data + "\""); }
  }
}