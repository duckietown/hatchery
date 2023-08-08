package org.duckietown.hatchery.achdjian.data

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.diagnostic.Logger
import java.nio.file.*
import kotlin.streams.toList

class RosEnvironments {
    var versions: MutableList<RosVersionImpl> = ArrayList()

    val customVerison: RosCustomVersion by lazy {
        ApplicationManager.getApplication().getComponent(RosCustomVersion::class.java)
    }

    private val defaultVersionsName: List<String> by lazy {
        val LOG = Logger.getInstance(RosEnvironments::class.java.name)
        if (Files.exists(Paths.get("/opt/ros"))) {
            val defaultVersions = Files.list(Paths.get("/opt/ros")).toList()
            LOG.trace("defaultVersionsName: " + defaultVersionsName.joinToString(","))

            val versions = HashMap<String, String>()
            defaultVersions.associateByTo(versions, { it.fileName.toString() }, { it.toString() })

            LOG.trace("defaultVersionToRemove:" + customVerison.defaultVersionToRemove.joinToString("m"))
            customVerison.defaultVersionToRemove.forEach { versions.remove(it) }

            LOG.trace("custom versions:" + customVerison.versions)
            customVerison.versions.forEach { (key, value) -> versions[key] = value }

            LOG.trace("ROS versions:" + customVerison.versions)
            this.versions.addAll(versions.map { RosVersionImpl(it.value, it.key) }.toList())

            defaultVersions.map { it.fileName.toString() }
        } else {
            listOf()
        }
    }

    fun isDefaultVersion(versionName: String) = defaultVersionsName.contains(versionName)

    fun contains(rosVersion: RosVersionImpl) = versions.contains(rosVersion)

    fun add(rosVersion: RosVersionImpl) = versions.add(rosVersion)

    fun remove(rosVersion: RosVersionImpl) = versions.removeIf { it.name == rosVersion.name }

    fun getOwnerVersion(path: Path) = versions.firstOrNull { path.startsWith(it.path) }
}

fun getRosEnvironment() = ApplicationManager.getApplication().getComponent(RosEnvironments::class.java)