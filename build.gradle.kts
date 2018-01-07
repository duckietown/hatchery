import com.sun.org.glassfish.external.amx.AMXUtil.prop
import de.undercouch.gradle.tasks.download.Download
import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
//import org.jetbrains.grammarkit.GrammarKitPluginExtension
import org.jetbrains.intellij.tasks.RunIdeaTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

//buildscript {
//    repositories {
//        maven { setUrl("https://jitpack.io") }
//    }
//    dependencies {
//        classpath("com.github.hurricup:gradle-grammar-kit-plugin:2017.1.1")
//    }
//}

plugins {
    idea
    kotlin("jvm") version "1.2.0"
    id("org.jetbrains.intellij") version "0.2.17"
    id("de.undercouch.download") version "3.2.0"
}

idea {
    module {
        excludeDirs = excludeDirs + file("deps")
    }
}

val clionVersion = "2017.3.1"

val setupClion = task<Download>("setupClion") {
    download {
        src("https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz")
        dest(file("${project.projectDir}/deps/clion-$clionVersion.tar.gz"))
    }
    copy {
        from(tarTree("deps/clion-$clionVersion.tar.gz"))
        into(file("${project.projectDir}/deps"))
    }
}

tasks.withType<RunIdeaTask> {
    dependsOn(setupClion)
    if (hasProperty("roject"))
        args = listOf(getProperty("roject") as String)
    else if (System.getenv().containsKey("DUCKIETOWN_ROOT"))
        args = listOf(System.getenv("DUCKIETOWN_ROOT"))
}

//configure<GrammarKitPluginExtension> {
//    grammarKitRelease = "1.5.2"
//}

intellij {
    pluginName = "hatchery"
    updateSinceUntilBuild = false
    if (hasProperty("roject")) downloadSources = false
    alternativeIdePath = "deps/clion-$clionVersion"

    setPlugins("BashSupport:1.6.12.173",                 // Shell syntax support
            "name.kropp.intellij.makefile:1.2.1",        // Makefile support
            "org.intellij.plugins.markdown:173.2696.26", // Markdown support
            "net.seesharpsoft.intellij.plugins.csv:1.3", // CSV file support
            "com.intellij.ideolog:173.0.5.0",            // Log file support
            "yaml")
}

group = "edu.umontreal"
version = "0.2"