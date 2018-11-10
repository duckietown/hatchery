package edu.umontreal.hatchery.settings

import edu.umontreal.hatchery.RosInstall
import java.io.File
import kotlin.reflect.KProperty

data class RosSettings(var rosPath: String = RosInstall.installDir) {
  // Force delegate to read the most current value by invoking as a function
  operator fun (() -> String).getValue(s: RosSettings, p: KProperty<*>) = File(this())
}