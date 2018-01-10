import com.sun.org.glassfish.external.amx.AMXUtil.prop
import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
import org.jetbrains.intellij.tasks.RunIdeaTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

import org.jetbrains.grammarkit.GrammarKitPluginExtension
import org.jetbrains.grammarkit.tasks.GenerateLexer
import org.jetbrains.grammarkit.tasks.GenerateParser

buildscript {
    repositories {
        maven { setUrl("https://jitpack.io") }
    }

    dependencies {
        classpath("com.github.hurricup:gradle-grammar-kit-plugin:2017.1.1")
    }
}

plugins {
    idea
    kotlin("jvm") version "1.2.10"
    id("org.jetbrains.intellij") version "0.2.17"
}

apply {
    plugin("idea")
    plugin("kotlin")
    plugin("org.jetbrains.grammarkit")
    plugin("org.jetbrains.intellij")
}

repositories {
    mavenCentral()
}

tasks.withType<RunIdeaTask> {
    if (hasProperty("roject"))
        args = listOf(getProperty("roject") as String)
    else if (System.getenv().containsKey("DUCKIETOWN_ROOT"))
        args = listOf(System.getenv("DUCKIETOWN_ROOT"))
}

configure<GrammarKitPluginExtension> {
    grammarKitRelease = "1.5.2"
}

val generateROSInterfaceLexer = task<GenerateLexer>("generateROSInterfaceLexer") {
    source = "src/main/grammars/ROSInterface.flex"
    targetDir = "src/main/java/edu/umontreal/hatchery/"
    targetClass = "ROSInterfaceLexer"
    purgeOldFiles = true
}

val generateROSInterfaceParser = task<GenerateParser>("generateROSInterfaceParser") {
    source = "src/main/grammars/ROSInterface.bnf"
    targetRoot = "src/main/java"
    pathToParser = "/edu/umontreal/hatchery/parser/ROSInterfaceParser.java"
    pathToPsiRoot = "/edu/umontreal/hatchery/psi"
    purgeOldFiles = true
}

tasks.withType<KotlinCompile> {
    dependsOn(generateROSInterfaceLexer, generateROSInterfaceParser)
}

intellij {
    pluginName = "hatchery"
    updateSinceUntilBuild = false
    if (hasProperty("roject")) downloadSources = false

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
version = "0.2.1"