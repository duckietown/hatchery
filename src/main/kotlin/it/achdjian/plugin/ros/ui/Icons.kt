package it.achdjian.plugin.ros.ui

import com.intellij.icons.AllIcons
import com.intellij.openapi.util.IconLoader
import com.intellij.openapi.util.NotNullLazyValue
import javax.swing.Icon

object ICON_NODE : NotNullLazyValue<Icon>() {
    override fun compute() = IconLoader.findIcon("/icons/ros.svg") ?: AllIcons.Icon
}

object ICON_LAUNCH : NotNullLazyValue<Icon>() {
    override fun compute() = IconLoader.findIcon("/icons/ros.svg") ?: AllIcons.Icon
}