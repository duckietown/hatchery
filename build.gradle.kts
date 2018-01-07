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
    //idea
    kotlin("jvm") version "1.2.0"
    id("org.jetbrains.intellij") version "0.2.17"
    id("de.undercouch.download") version "3.2.0"
}

//idea {
//    module {
//        excludeDirs = excludeDirs + file("deps")
//    }
//}

val clionVersion = "2017.3.1"

//val setupClion = task<Download>("setupClion") {
//    download {
//        onlyIf { !file("${project.projectDir}/deps/clion-$clionVersion.tar.gz").exists() }
//        src("https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz")
//        dest(file("${project.projectDir}/deps/clion-$clionVersion.tar.gz"))
//    }
//    copy {
//        onlyIf { !file("${project.projectDir}/deps/clion-$clionVersion").exists() }
//        from(tarTree("deps/clion-$clionVersion.tar.gz"))
//        into(file("${project.projectDir}/deps"))
//    }
//}

val downloadClion = task<Download>("downloadClion") {
    onlyIf { !file("${project.projectDir}/.gradle/deps/clion-$clionVersion.tar.gz").exists() }
    src("https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz")
    dest(file("${project.projectDir}/.gradle/deps/clion-$clionVersion.tar.gz"))
}
val unpackClion = task<Copy>("unpackClion") {
    onlyIf { !file("${project.projectDir}/deps/clion-$clionVersion").exists() }
    from(tarTree("deps/clion-$clionVersion.tar.gz"))
    into(file("${project.projectDir}/deps"))
    dependsOn(downloadClion)
}

tasks.withType<RunIdeaTask> {
    // dependsOn(unpackClion)
    if (hasProperty("roject"))
        args = listOf(getProperty("roject") as String)
    else if (System.getenv().containsKey("DUCKIETOWN_ROOT")) {
	//val cmakelists_path = File().walkTopDown().first { it.endsWith("CMakeLists.txt") }.toString()
        args = listOf(System.getenv("DUCKIETOWN_ROOT") /*+ "/catkin_ws/src/"*/)
    }
}

//configure<GrammarKitPluginExtension> {
//    grammarKitRelease = "1.5.2"
//}

intellij {
    pluginName = "hatchery"
    updateSinceUntilBuild = false
    if (hasProperty("roject")) downloadSources = false
    //alternativeIdePath = "deps/clion-$clionVersion"

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
version = "0.2"
