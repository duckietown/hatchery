package org.duckietown.hatchery.psi

import com.intellij.psi.PsiNameIdentifierOwner

interface RosInterfaceNamedElement : PsiNameIdentifierOwner {
  // https://github.com/ros2/ros2/wiki/About-ROS-Interfaces#212-field-names
  fun getKey(): String?

  // https://github.com/ros2/ros2/wiki/About-ROS-Interfaces#213-field-default-value
  fun getValue(): String?

  // https://github.com/ros2/ros2/wiki/About-ROS-Interfaces#211-field-types
  fun getType(): String?
}