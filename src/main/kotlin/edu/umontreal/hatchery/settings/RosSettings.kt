package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.ros.Ros
import edu.umontreal.hatchery.ros.defaultRosSetupScript
import java.io.File
import kotlin.reflect.KProperty

class RosSettings {
  var localRunCommand: String = "roslaunch"
  var remoteAddress: String = ""

  var remoteRunCommand: String = "roslaunch"
  var sshCredentialsPath: String = ""

  var localRosPath = defaultRosSetupScript
    set(value) {
      localRos = Ros(value ?: "")
      field = value
    }

  var localRos = Ros()
    private set

  var remoteRosPath: String = defaultRosSetupScript ?: ""
    set(value) {
      remoteRos = Ros(value)
      field = value
    }

  var remoteRos = Ros()
    private set

  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}