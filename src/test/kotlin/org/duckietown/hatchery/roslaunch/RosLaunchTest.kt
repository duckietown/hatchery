package org.duckietown.hatchery.roslaunch

import com.intellij.testFramework.fixtures.BasePlatformTestCase
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

//  fun testPythonInterpreter() {
//    Context.create().use { it.eval("python", """
//      import sys
//      import os
//      print(os.environ)
//      a = 2+2
//      print(a)
//    """.trimIndent()) }
//  }
}
