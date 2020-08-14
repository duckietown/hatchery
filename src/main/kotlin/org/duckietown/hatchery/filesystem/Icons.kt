package org.duckietown.hatchery.filesystem

import com.intellij.openapi.util.IconLoader

object Icons {
  val ros_launch by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/hatching.svg") }
  val resource_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/duckling.svg") }
  val package_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/packagefile.svg") }
  val broken_resource by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/broken_resource.svg") }
  val ros_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/bat.png") }
  val launch_dir by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/egg.svg") }
  val catkin_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/stork.svg") }
  val workspace by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/duck.svg") }
  val ros_msg by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/dove.svg") }
  val python_dir by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/parrot.svg") }
}