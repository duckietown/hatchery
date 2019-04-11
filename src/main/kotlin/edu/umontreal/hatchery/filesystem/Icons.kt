package edu.umontreal.hatchery.filesystem

import com.intellij.openapi.util.IconLoader

object Icons {
  val ros_launch by lazy { IconLoader.getIcon("/icons/hatching.svg") }
  val resource_file by lazy { IconLoader.getIcon("/icons/duckling.svg") }
  val package_file by lazy { IconLoader.getIcon("/icons/packagefile.svg") }
  val broken_resource by lazy { IconLoader.getIcon("/icons/broken_resource.svg") }
  val ros_file by lazy { IconLoader.getIcon("/icons/bat.png") }
  val launch_dir by lazy { IconLoader.getIcon("/icons/egg.svg") }
  val catkin_file by lazy { IconLoader.getIcon("/icons/stork.svg") }
  val workspace by lazy { IconLoader.getIcon("/icons/duck.svg") }
  val ros_msg by lazy { IconLoader.getIcon("/icons/dove.svg") }
  val python_dir by lazy { IconLoader.getIcon("/icons/parrot.svg") }
}