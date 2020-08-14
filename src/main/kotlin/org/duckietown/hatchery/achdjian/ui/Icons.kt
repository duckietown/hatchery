package org.duckietown.hatchery.achdjian.ui

import com.intellij.ide.ui.ProductIcons
import com.intellij.openapi.util.*
import javax.swing.Icon

object ICON_NODE : NotNullLazyValue<Icon>() {
    override fun compute() = IconLoader.findIcon("/org/duckietown/hatchery/icons/ros.svg") ?: ProductIcons.getInstance().productIcon
}

object ICON_LAUNCH : NotNullLazyValue<Icon>() {
    override fun compute() = IconLoader.findIcon("/org/duckietown/hatchery/icons/ros.svg") ?: ProductIcons.getInstance().productIcon
}