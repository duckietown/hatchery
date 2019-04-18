package it.achdjian.plugin.ros.data

import com.intellij.openapi.components.PersistentStateComponent
import com.intellij.openapi.components.State
import com.intellij.openapi.components.Storage

@State(name = "ROS.configuration", storages = [(Storage("ROS.xml"))])
data class RosCustomVersion(var versions: MutableMap<String, String>) : PersistentStateComponent<RosCustomVersion> {
    var defaultVersionToRemove = HashSet<String>()

    constructor() : this(HashMap())

    override fun getState(): RosCustomVersion? {
        return this
    }

    override fun loadState(state: RosCustomVersion) {
        versions.clear()
        versions.putAll(state.versions)
        defaultVersionToRemove.clear()
        defaultVersionToRemove.addAll(state.defaultVersionToRemove)
    }

    fun contains(version: RosVersionImpl) = versions.containsKey(version.name)
    fun remove(version: RosVersionImpl) = versions.remove(version.name)
    fun removeDefault(versionName: String) = defaultVersionToRemove.add(versionName)
}
