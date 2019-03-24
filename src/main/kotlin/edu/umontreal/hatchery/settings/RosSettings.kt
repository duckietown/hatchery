package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.ros.defaultRosSetupScript
import java.io.File
import kotlin.reflect.KProperty

data class RosSettings(
  var localRosPath: String = defaultRosSetupScript,
  var localRunCommand: String = "roslaunch",
  var remoteAddress: String = "",
  var remoteRosPath: String = defaultRosSetupScript,
  var remoteRunCommand: String = "roslaunch",
  var sshCredentialsPath: String = "") {
  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}