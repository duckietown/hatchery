package org.duckietown.hatchery.filesystem

import com.intellij.openapi.util.IconLoader

object Icons {
  val cl = ClassLoader.getPlatformClassLoader()
  val ros_launch by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/hatching.svg", cl) }
  val resource_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/duckling.svg", cl) }
  val package_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/packagefile.svg", cl) }
  val broken_resource by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/broken_resource.svg", cl) }
  val ros by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/ros.svg", cl) }
  val ros_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/bat.png", cl) }
  val ros_folder by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/rosFolder.svg", cl) }
  val launch_dir by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/egg.svg", cl) }
  val catkin_file by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/stork.svg", cl) }
  val workspace by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/duck.svg", cl) }
  val ros_msg by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/dove.svg", cl) }
  val python_dir by lazy { IconLoader.getIcon("/org/duckietown/hatchery/icons/parrot.svg", cl) }
}