package edu.umontreal.hatchery.roslaunch

import com.intellij.testFramework.UsefulTestCase
import com.intellij.testFramework.fixtures.LightPlatformCodeInsightFixtureTestCase

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
}