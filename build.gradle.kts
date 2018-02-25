import com.sun.org.glassfish.external.amx.AMXUtil.prop
import de.undercouch.gradle.tasks.download.Download
import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
import org.gradle.language.base.internal.plugins.CleanRule
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
    classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.2.21")
    classpath("com.github.hurricup:gradle-grammar-kit-plugin:2017.1.1")
  }
}

plugins {
  idea
  kotlin("jvm") version "1.2.21"
  id("org.jetbrains.intellij") version "0.2.18"
  id("de.undercouch.download") version "3.2.0"
}

apply {
  idea
  kotlin
  plugin("org.jetbrains.grammarkit")
  plugin("org.jetbrains.intellij")
  plugin("de.undercouch.download")
}

repositories {
  mavenCentral()
}

val clionVersion = "2017.3.2"

val downloadClion = task<Download>("downloadClion") {
  onlyIf { !file("${project.projectDir}/build/clion/clion-$clionVersion.tar.gz").exists() }
  src("https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz")
  dest(file("${project.projectDir}/build/clion/clion-$clionVersion.tar.gz"))
}

val unpackClion = task<Copy>("unpackClion") {
  onlyIf { !file("${project.projectDir}/build/clion/build/clion/clion-$clionVersion").exists() }
  from(tarTree("build/clion/clion-$clionVersion.tar.gz"))
  into(file("${project.projectDir}/build/clion"))
  dependsOn(downloadClion)
}

tasks.withType<RunIdeaTask> {
  dependsOn(unpackClion)

  var projectRoot = ""
  if (hasProperty("roject"))
    projectRoot = getProperty("roject") as String
  else if (System.getenv().containsKey("DUCKIETOWN_ROOT"))
    projectRoot = System.getenv("DUCKIETOWN_ROOT")

  if(projectRoot.isNotEmpty()){
    projectRoot += "/catkin_ws/src/CMakeLists.txt"
    println("Project root directory: $projectRoot")
    args = listOf(projectRoot)
  }
}

configure<GrammarKitPluginExtension> { grammarKitRelease = "1.5.2" }

val generateROSInterfaceLexer = task<GenerateLexer>("generateROSInterfaceLexer") {
  source = "src/main/grammars/ROSInterface.flex"
  targetDir = "src/main/java/edu/umontreal/hatchery/rosinterface"
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
  version = clionVersion
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false
  alternativeIdePath = "build/clion/clion-$clionVersion"

  setPlugins("name.kropp.intellij.makefile:1.2.2", // Makefile support
      "BashSupport:1.6.12.173",                    // Shell syntax support
      "org.intellij.plugins.markdown:173.2696.26", // Markdown support
      "net.seesharpsoft.intellij.plugins.csv:1.3", // CSV file support
      "com.intellij.ideolog:173.0.6.0",            // Log file support
      "PsiViewer:3.28.93",                         // PSI view support
      "yaml")                                      // YML file support
}

group = "edu.umontreal"
version = "0.2.1"
