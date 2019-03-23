package edu.umontreal.hatchery.roslaunch

import com.intellij.testFramework.fixtures.LightPlatformCodeInsightFixtureTestCase
import org.python.core.PyInteger
import org.python.util.PythonInterpreter

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
    val completionResults = myFixture.completeBasic()
  }

  fun testPythonInterpreter() {
    val interp = PythonInterpreter()

    println("Hello, brave new world")
    interp.exec("import sys")
    interp.exec("print sys")

    interp.set("a", PyInteger(42))
    interp.exec("print a")
    interp.exec("x = 2+2")
    val x = interp.get("x")

    println("x: $x")
    println("Goodbye, cruel world")
  }
}