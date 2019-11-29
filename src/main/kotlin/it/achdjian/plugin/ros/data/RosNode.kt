package it.achdjian.plugin.ros.data

import java.nio.file.Path

class RosNode(val path: Path) {
    val name: String
        get() = path.fileName.toString()
}