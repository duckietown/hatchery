package org.duckietown.hatchery.roslaunch

import com.intellij.testFramework.fixtures.BasePlatformTestCase
import org.python.core.PyInteger
import org.python.util.PythonInterpreter
import java.util.*

/**
 * Tests roslaunch functionality.
 */

class RosLaunchTest : BasePlatformTestCase() {
  fun `test launch file detected`() {
    myFixture.configureByText(RosLaunchFileType, """
        <launch>
        <node name="talker" pkg="rospy_tutorials" type="talker" />
        <caret>
        </launch>
      """.trimIndent())
  }

  fun testPythonInterpreter() {
    val props = Properties().apply {
      setProperty("python.path", "/home/modules:scripts")
    }
    PythonInterpreter.initialize(System.getProperties(), props, arrayOf(""))
    PythonInterpreter().run {
      exec("import sys")
      exec("print sys")

      set("a", PyInteger(42))
      exec("print a")
      exec("x = 2+2")
      val x = get("x")

      println("x: $x")
    }
  }
}