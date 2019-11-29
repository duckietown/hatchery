package org.duckietown.hatchery.ros.nodes

import org.ros.concurrent.CancellableLoop
import org.ros.namespace.GraphName
import org.ros.node.AbstractNodeMain
import org.ros.node.ConnectedNode

class Talker: AbstractNodeMain() {
  override fun getDefaultNodeName() = GraphName.of("Talker")!!

  override fun onStart(connectedNode: ConnectedNode?) {
    val publisher = connectedNode!!.newPublisher<std_msgs.String>("chatter", "std_msgs/String")

    connectedNode.executeCancellableLoop(object: CancellableLoop() {
      private var sequence = 0

      override fun setup() {
        sequence = 0
      }

      @Throws(InterruptedException::class)
      override fun loop() {
        println(sequence)
        val str = publisher.newMessage()
        str.data = "Hello World$sequence"
        publisher.publish(str)
        sequence++
        Thread.sleep(1000)
      }
    })
  }
}