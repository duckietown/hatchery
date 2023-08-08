package org.duckietown.hatchery.achdjian.ui

import com.intellij.openapi.util.*
import org.duckietown.hatchery.filesystem.Icons
import javax.swing.Icon

object ICON_NODE : NotNullLazyValue<Icon>() {
    override fun compute() = Icons.ros
}

object ICON_LAUNCH : NotNullLazyValue<Icon>() {
    override fun compute() = Icons.ros
}