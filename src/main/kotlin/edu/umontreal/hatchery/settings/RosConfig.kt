package edu.umontreal.hatchery.settings

import com.intellij.openapi.components.*
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.options.Configurable

/* Persists the state of the AceJump IDE settings across IDE restarts.
 * https://www.jetbrains.org/intellij/sdk/docs/basics/persisting_state_of_components.html
 */

@State(name = "RosConfig", storages = [(Storage("hatchery.xml"))])
object RosConfig : Configurable, PersistentStateComponent<RosSettings> {
  private val logger = Logger.getInstance(RosConfig::class.java)
  var settings: RosSettings = RosSettings()

  override fun getState() = settings

  override fun loadState(state: RosSettings) {
    logger.info("Loaded RosConfig settings: $settings")
    settings = state
  }

  private val panel = RosSettingsPanel()

  override fun getDisplayName() = "Hatchery"

  override fun createComponent() = panel.rootPanel

  override fun isModified() = panel.localRosPath != settings.localRosPath ||
    panel.remoteAddress != settings.remoteAddress

  override fun apply() {
    settings.localRosPath = panel.localRosPath
    settings.remoteRosPath = panel.remoteRosPath
    settings.remoteAddress = panel.remoteAddress
    settings.sshCredentialsPath = panel.sshCredentialsPath
    logger.info("User applied new settings: $settings")
  }

  override fun reset() = panel.reset(settings)
}