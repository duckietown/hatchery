package it.achdjian.plugin.ros.ui

import com.intellij.ide.ui.ProductIcons
import com.intellij.openapi.util.IconLoader
import com.intellij.openapi.util.NotNullLazyValue
import javax.swing.Icon

object ICON_NODE : NotNullLazyValue<Icon>() {
  override fun compute() = IconLoader.findIcon("/icons/ros.svg") ?: ProductIcons.getInstance().productIcon
}

object ICON_LAUNCH : NotNullLazyValue<Icon>() {
  override fun compute() = IconLoader.findIcon("/icons/ros.svg") ?: ProductIcons.getInstance().productIcon
}