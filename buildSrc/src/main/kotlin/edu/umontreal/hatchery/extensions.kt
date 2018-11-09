package edu.umontreal.hatchery

import org.gradle.api.*

/**
 * Defines an extension property of Project that can be used in the build script of the project (or any subproject)
 */
val Project.profile get() = findProperty("profile") ?: "dev"

/**
 * Defines an extension function of Project that can be used in the build script of the project (or any subproject)
 */
fun Project.logProfile() = println("Here's the profile: $profile")