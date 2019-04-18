package it.achdjian.plugin.ros.settings

import java.io.File

data class InitWorkspaceCmd(val executableFile: File, val args: String?) {
  override fun toString() = "$executableFile $args".trim()
}


