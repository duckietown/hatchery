package org.duckietown.hatchery.settings

import com.intellij.openapi.components.*
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.options.Configurable

/* Persists the state of the AceJump IDE settings across IDE restarts.
 * https://www.jetbrains.org/intellij/sdk/docs/basics/persisting_state_of_components.html
 */

@State(name = "RosConfig", storages = [(Storage("hatchery.xml"))])
class RosConfig : Configurable, PersistentStateComponent<RosSettings> {
  private val logger = Logger.getInstance(RosConfig::class.java)

  companion object {
    val rosSettings: RosSettings
      get() = ServiceManager.getService(RosConfig::class.java).settings
  }

  var settings: RosSettings = RosSettings()
    private set

  override fun getState() = settings

  override fun loadState(state: RosSettings) {
    logger.info("Loaded RosConfig settings: $settings")
    settings = state
  }

  private val panel by lazy { RosSettingsPanel() }

  override fun getDisplayName() = "Hatchery"

  override fun createComponent() = panel.rootPanel

  override fun isModified() = panel.localRosPath != settings.localRosPath ||
    panel.remoteAddress != settings.remoteAddress ||
    panel.defaultRosLaunchOptions != settings.defaultRosLaunchOptions

  override fun apply() {
    settings.localRosPath = panel.localRosPath
    settings.defaultRosLaunchOptions = panel.defaultRosLaunchOptions
    settings.remoteRosPath = panel.remoteRosPath
    settings.remoteAddress = panel.remoteAddress
    settings.remoteRunCommand = panel.remoteRunCommand
    settings.sshCredentialsPath = panel.sshCredentialsPath
    logger.info("User applied new settings: $settings")
  }

  override fun reset() = panel.reset(settings)
}