package org.duckietown.hatchery.achdjian.data

import com.intellij.openapi.components.*

@State(name = "ROS.configuration", storages = [(Storage("ROS.xml"))])
data class RosCustomVersion(var versions: MutableMap<String, String> = HashMap()) : PersistentStateComponent<org.duckietown.hatchery.achdjian.data.RosCustomVersion> {
    var defaultVersionToRemove = HashSet<String>()

    override fun getState() = this

    override fun loadState(state: org.duckietown.hatchery.achdjian.data.RosCustomVersion) {
        versions.clear()
        versions.putAll(state.versions)
        defaultVersionToRemove.clear()
        defaultVersionToRemove.addAll(state.defaultVersionToRemove)
    }

    fun contains(version: org.duckietown.hatchery.achdjian.data.RosVersionImpl) = versions.containsKey(version.name)
    fun remove(version: org.duckietown.hatchery.achdjian.data.RosVersionImpl) = versions.remove(version.name)
    fun removeDefault(versionName: String) = defaultVersionToRemove.add(versionName)
}
