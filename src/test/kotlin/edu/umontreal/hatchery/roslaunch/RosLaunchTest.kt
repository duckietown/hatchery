package edu.umontreal.hatchery.roslaunch

import com.intellij.testFramework.fixtures.LightPlatformCodeInsightFixtureTestCase
import org.python.core.PyInteger
import org.python.util.PythonInterpreter
import java.util.*

/**
 * Tests roslaunch functionality.
 */

class RosLaunchTest: LightPlatformCodeInsightFixtureTestCase() {
  fun `test launch file detected`() {
    myFixture.configureByText(RosLaunchFileType, """
        <launch>
        <node name="talker" pkg="rospy_tutorials" type="talker" />
        <caret>
        </launch>
      """.trimIndent())
  }

  fun testPythonInterpreter() {
    val props = Properties()
    props.setProperty("python.path", "/home/modules:scripts")
    PythonInterpreter.initialize(System.getProperties(), props, arrayOf(""))
    val interp = PythonInterpreter()

    println("Hello, brave new world")
    interp.exec("import rospy")
    interp.exec("print sys")

    interp.set("a", PyInteger(42))
    interp.exec("print a")
    interp.exec("x = 2+2")
    val x = interp.get("x")

    println("x: $x")
    println("Goodbye, cruel world")
  }
}