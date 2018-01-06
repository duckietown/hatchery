import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
import org.jetbrains.intellij.tasks.RunIdeaTask

tasks.withType<RunIdeaTask> {
    if(hasProperty("roject"))
        args = listOf(getProperty("roject") as String)
    else if (System.getenv().containsKey("DUCKIETOWN_ROOT"))
        args  = listOf(System.getenv("DUCKIETOWN_ROOT"))
}

plugins {
    kotlin("jvm") version "1.2.0"
    id("org.jetbrains.intellij") version "0.2.17"
}

intellij {
    pluginName = "hatchery"
    updateSinceUntilBuild = false
    if(hasProperty("roject")) downloadSources = false

    setPlugins("PythonCore:2017.3.173.4127.35",          // Python support
            "name.kropp.intellij.makefile:1.2.1",        // Makefile support
            "artsiomch.cmake:0.1.0",                     // CMake syntax support
            "BashSupport:1.6.12.173",                    // Shell syntax support
            "nl.rubensten.texifyidea:0.5",               // LaTeX syntax support
            "org.intellij.plugins.markdown:173.2696.26", // Markdown support
            "net.seesharpsoft.intellij.plugins.csv:1.3", // CSV file support
            "com.intellij.ideolog:173.0.5.0",            // Log file support
            "yaml")
}

group = "edu.umontreal"
version = "1.0-SNAPSHOT"