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

    setPlugins("PythonCore:2017.3.173.4127.35", "BashSupport:1.6.12.173")
}

group = "com.johnlindquist"
version = "3.4.3"