package it.achdjian.plugin.ros.settings

import com.intellij.openapi.diagnostic.Logger
import it.achdjian.plugin.ros.utils.getEnvironment
import java.io.File
import java.nio.file.Path

fun findInitCmd(path: String): InitWorkspaceCmd? {
    val log = Logger.getInstance("#it.achdjian.plugin.ros.settings.RosSettingsUtils.findInitCmd")
    return path.split(":")
            .map { it -> File(it, "catkin_init_workspace") }
            .map{it->log.trace("Search for " + it.absolutePath + ":  exists-> " + it.exists()); it}
            .firstOrNull { it.exists() }?.let {
                InitWorkspaceCmd(it, "")
            }
}

fun diffEnvironment(rosVersion: Path): Map<String, String> {
    val log = Logger.getInstance("#it.achdjian.plugin.ros.settings.RosSettingsUtils.diffEnvironment")

    val actualEnv = System.getenv()

    val newEnv = getEnvironment(rosVersion, rosVersion.toAbsolutePath().toString() + "/setup.bash")
    val env = HashMap<String,String>(diff(newEnv, actualEnv))
    log.trace("Diff env:")
    env.forEach { key, value -> log.trace("$key=$value") }
    if (!env.containsKey("ROS_PACKAGE_PATH")) {
        if (actualEnv.containsKey("ROS_PACKAGE_PATH")) {
            env["ROS_PACKAGE_PATH"] = actualEnv["ROS_PACKAGE_PATH"] as String
        } else {
            env["ROS_PACKAGE_PATH"] = rosVersion.toAbsolutePath().toString()+"/share"
        }
    }
    env["PATH"] = newEnv["PATH"] as String
    return env
}

fun diff(newEnv: Map<String, String>, actualEnv: Map<String, String>) =
        newEnv.filter { (key, value) ->
            !actualEnv.containsKey(key) || !actualEnv[key].equals(value)
        }

