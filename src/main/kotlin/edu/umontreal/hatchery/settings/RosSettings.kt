package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.ros.Ros
import java.io.File
import kotlin.reflect.KProperty

class RosSettings {
  var defaultRosLaunchOptions: String = ""
  var remoteAddress: String = ""

  var remoteRunCommand: String = "roslaunch"
  var sshCredentialsPath: String = ""

  var localRosPath = ""
    set(value) {
      localRos = Ros(value ?: "")
      field = value
    }

  lateinit var localRos: Ros
    private set

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