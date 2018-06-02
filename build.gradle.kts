import de.undercouch.gradle.tasks.download.Download
import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
import org.gradle.language.base.internal.plugins.CleanRule
import org.jetbrains.intellij.tasks.RunIdeTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import org.jetbrains.grammarkit.GrammarKitPluginExtension
import org.jetbrains.grammarkit.tasks.GenerateLexer
import org.jetbrains.grammarkit.tasks.GenerateParser
import org.jetbrains.kotlin.backend.common.onlyIf

buildscript {
  repositories {
    maven { setUrl("https://jitpack.io") }
  }

  dependencies {
    classpath("com.github.hurricup:gradle-grammar-kit-plugin:2018.1.2")
  }
}

plugins {
  idea apply true
  kotlin("jvm") version "1.2.41" apply true
  id("org.jetbrains.intellij") version "0.3.1" apply true
  id("de.undercouch.download") version "3.4.3" apply true
}

apply {
  plugin("org.jetbrains.grammarkit")
}

repositories {
  mavenCentral()
}

val clionVersion = "2018.1.3"
val installPath = "${project.projectDir}/build/clion/clion-$clionVersion"
val downloadURL = "https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz"

tasks {
  val downloadClion = "downloadClion"(Download::class) {
    onlyIf { !file("$installPath.tar.gz").exists() }
    src(downloadURL)
    dest(file("$installPath.tar.gz"))
  }

  val unpackClion = "unpackClion"(Copy::class) {
    onlyIf { !file(installPath).exists() }
    from(tarTree("build/clion/clion-$clionVersion.tar.gz"))
    into(file("${project.projectDir}/build/clion"))
    dependsOn(downloadClion)
  }

  withType<RunIdeTask> {
    dependsOn(unpackClion)
    var projectRoot = ""
    if (hasProperty("roject"))
      projectRoot = getProperty("roject") as String
    else if (System.getenv().containsKey("DUCKIETOWN_ROOT"))
      projectRoot = System.getenv("DUCKIETOWN_ROOT")

    if (projectRoot.isNotEmpty()) {
      projectRoot += "/catkin_ws/src/CMakeLists.txt"
      println("Project root directory: $projectRoot")
      args = listOf(projectRoot)
    }
  }

  val generateROSInterfaceLexer = "generateLexer"(GenerateLexer::class) {
    source = "src/main/grammars/ROSInterface.flex"
    targetDir = "src/main/java/edu/umontreal/hatchery/rosinterface"
    targetClass = "ROSInterfaceLexer"
    purgeOldFiles = true
  }

  val generateROSInterfaceParser = "generateParser"(GenerateParser::class) {
    source = "src/main/grammars/ROSInterface.bnf"
    targetRoot = "src/main/java"
    pathToParser = "/edu/umontreal/hatchery/parser/ROSInterfaceParser.java"
    pathToPsiRoot = "/edu/umontreal/hatchery/psi"
    purgeOldFiles = true
  }

  withType<KotlinCompile> {
    kotlinOptions.jvmTarget = "1.8"
    dependsOn(generateROSInterfaceLexer, generateROSInterfaceParser)
  }
}

intellij {
  pluginName = "hatchery"
  version = clionVersion
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false
  alternativeIdePath = "build/clion/clion-$clionVersion"

  setPlugins("name.kropp.intellij.makefile:1.2.2", // Makefile support
      "org.intellij.plugins.markdown:181.4668.12", // Markdown support
      "net.seesharpsoft.intellij.plugins.csv:1.3", // CSV file support
      "com.intellij.ideolog:181.0.7.0",            // Log file support
      "BashSupport:1.6.13.181",                    // Shell syntax support
      "Docker:181.4668.68",                        // Docker support
      "PsiViewer:2018.1",                          // PSI view support
      "yaml")                                      // YML file support
}

group = "edu.umontreal"
version = "0.2.1"