package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.ros.Ros
import edu.umontreal.hatchery.ros.defaultRosSetupScript
import java.io.File
import kotlin.reflect.KProperty

class RosSettings {
  var localRunCommand: String = "roslaunch"
  var remoteAddress: String = ""
  var remoteRosPath: String = defaultRosSetupScript
  var remoteRunCommand: String = "roslaunch"
  var sshCredentialsPath: String = ""

  var localRosPath = defaultRosSetupScript
    set(value) {
      ros = Ros(localRosPath)
      field = value
    }

  var ros = Ros(localRosPath)
    private set

  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}