package edu.umontreal.hatchery.filesystem

import com.intellij.openapi.util.IconLoader

object Icons {
    val launch_file by lazy { IconLoader.getIcon("/icons/launch_icon.png") }
    val resource_file by lazy { IconLoader.getIcon("/icons/resource_icon.png") }
    val package_file by lazy { IconLoader.getIcon("/icons/package_icon.png") }
    val broken_resource by lazy { IconLoader.getIcon("/icons/broken_resource.png") }
    val ros_file by lazy { IconLoader.getIcon("/icons/ros_icon.png") }
    val catkin_file by lazy { IconLoader.getIcon("/icons/package_folder.png") }
    val ros_msg by lazy { IconLoader.getIcon("/icons/ros_message.png") }
    val robot by lazy { IconLoader.getIcon("/icons/robot_16.png") }
    val python_dir by lazy { IconLoader.getIcon("/icons/variables_16.png") }
}