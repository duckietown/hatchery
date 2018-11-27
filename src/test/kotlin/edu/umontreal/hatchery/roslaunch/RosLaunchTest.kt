package edu.umontreal.hatchery.roslaunch

import com.intellij.testFramework.UsefulTestCase
import com.intellij.testFramework.fixtures.LightCodeInsightFixtureTestCase

/**
 * Tests roslaunch functionality.
 */

class RosLaunchTest: LightCodeInsightFixtureTestCase() {
  fun `test launch file detected`() {
    myFixture.configureByText(RosLaunchFileType, """
        <launch>
        <node name="talker" pkg="rospy_tutorials" type="talker" />
        <caret>
        </launch>
      """.trimIndent())
    val completionResults = myFixture.completeBasic()
    UsefulTestCase.assertNotEmpty(completionResults.toList())
  }
}