package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.ros.*
import java.io.File
import kotlin.reflect.KProperty

data class RosSettings(
  var localRosPath: String = Ros().installDir,
  var localRunCommand: String = "roslaunch",
  var remoteAddress: String = "",
  var remoteRosPath: String = Ros().installDir,
  var remoteRunCommand: String = "roslaunch",
  var sshCredentialsPath: String = "") {
  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}