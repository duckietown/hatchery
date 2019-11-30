package it.achdjian.plugin.ros.data

import com.intellij.openapi.components.*

@State(name = "ROS.configuration", storages = [(Storage("ROS.xml"))])
data class RosCustomVersion(var versions: MutableMap<String, String> = HashMap()) : PersistentStateComponent<RosCustomVersion> {
    var defaultVersionToRemove = HashSet<String>()

    override fun getState() = this

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
