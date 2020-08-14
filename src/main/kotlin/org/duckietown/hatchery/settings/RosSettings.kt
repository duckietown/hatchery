package org.duckietown.hatchery.settings

import org.duckietown.hatchery.ros.Ros
import java.io.File
import kotlin.reflect.KProperty

class RosSettings {
  var defaultRosLaunchOptions: String = ""
  var remoteAddress: String = ""

  var remoteRunCommand: String = "roslaunch"
  var sshCredentialsPath: String = ""

  var localRosPath = ""
    set(value) {
      ros = Ros(value)
      field = value
    }

  var ros: Ros? = null
    get() {
      if (field == null) field = Ros()
      return field
    }
    private set

  val localRos: Ros
    get() = ros!!

  var remoteRosPath: String = ""
    set(value) {
      remoteRos = Ros(value)
      field = value
    }

  lateinit var remoteRos: Ros
    private set

  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}