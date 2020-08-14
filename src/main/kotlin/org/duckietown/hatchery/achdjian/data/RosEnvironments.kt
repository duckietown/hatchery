package org.duckietown.hatchery.achdjian.data

import com.intellij.openapi.application.ApplicationManager
import com.intellij.openapi.diagnostic.Logger
import java.nio.file.*
import kotlin.streams.toList

class RosEnvironments(customVerison: org.duckietown.hatchery.achdjian.data.RosCustomVersion) {
    companion object {
        val LOG = Logger.getInstance(RosEnvironments::class.java.name)
    }

    var versions: MutableList<RosVersionImpl> = ArrayList()
    private val defaultVersionsName: List<String>

    init {
        if (Files.exists(Paths.get("/opt/ros"))) {
            val defaultVersions = Files.list(Paths.get("/opt/ros")).toList()
            defaultVersionsName = defaultVersions.map { it.fileName.toString() }

            LOG.trace("defaultVersionsName: " + defaultVersionsName.joinToString(","))

            val versions = HashMap<String, String>()
            defaultVersions.associateByTo(versions, { it.fileName.toString() }, { it.toString() })

            LOG.trace("defaultVersionToRemove:" + customVerison.defaultVersionToRemove.joinToString("m"))
            customVerison.defaultVersionToRemove.forEach { versions.remove(it) }

            LOG.trace("custom versions:" + customVerison.versions)
            customVerison.versions.forEach { (key, value) -> versions[key] = value }

            LOG.trace("ROS versions:" + customVerison.versions)
            this.versions.addAll(versions.map { RosVersionImpl(it.value, it.key) }.toList())
        } else {
            defaultVersionsName = listOf()
        }
    }

    fun isDefaultVersion(versionName: String) = defaultVersionsName.contains(versionName)

    fun contains(rosVersion: RosVersionImpl) = versions.contains(rosVersion)

    fun add(rosVersion: RosVersionImpl) = versions.add(rosVersion)

    fun remove(rosVersion: RosVersionImpl) = versions.removeIf { it.name == rosVersion.name }

    fun getOwnerVersion(path: Path) = versions.firstOrNull { path.startsWith(it.path) }
}

fun getRosEnvironment() = ApplicationManager.getApplication().getComponent(RosEnvironments::class.java)