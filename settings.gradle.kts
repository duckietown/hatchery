rootProject.name = "hatchery"

pluginManagement.resolutionStrategy.eachPlugin {
  if (requested.id.id.startsWith("org.jetbrains.kotlin")) {
    gradle.rootProject.extra["kotlinVersion"]?.let { useVersion(it as String) }
  }
}