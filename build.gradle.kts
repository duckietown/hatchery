import de.undercouch.gradle.tasks.download.Download
import edu.umontreal.hatchery.withRosTask
import org.ajoberstar.grgit.Grgit
import org.apache.tools.ant.taskdefs.ExecTask
import org.gradle.api.tasks.JavaExec
import org.jetbrains.intellij.tasks.PublishTask
import org.gradle.language.base.internal.plugins.CleanRule
import org.jetbrains.gradle.ext.Application
import org.jetbrains.gradle.ext.GradleTask
import org.jetbrains.gradle.ext.ProjectSettings
import org.jetbrains.grammarkit.GrammarKit
import org.jetbrains.intellij.tasks.RunIdeTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import org.jetbrains.grammarkit.GrammarKitPluginExtension
import org.jetbrains.grammarkit.tasks.*
import org.jetbrains.kotlin.backend.common.onlyIf
import org.jetbrains.kotlin.cli.jvm.main
import kotlin.text.Typography.copyright

buildscript {
  repositories.maven("https://dl.bintray.com/kotlin/kotlin-eap")
  dependencies {
    val kotlinVersion = properties["kotlinVersion"]
    classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlinVersion")
  }
}

plugins {
  idea apply true
  kotlin("jvm")
  // TODO: https://github.com/JetBrains/gradle-python-envs#usage
  id("com.jetbrains.python.envs") version "0.0.25" apply true
  id("org.jetbrains.intellij") version "0.3.12" apply true
  id("de.undercouch.download") version "3.4.3" apply true
  id("org.jetbrains.grammarkit") version "2018.2.2" apply true
  id("org.ajoberstar.grgit") version "3.0.0-rc.2" apply true
  id("org.jetbrains.gradle.plugin.idea-ext") version "0.4.2" apply true
}

idea {
  module {
    isDownloadSources = true
    generatedSourceDirs.add(file("src/main/java"))
    excludeDirs.add(file(intellij.sandboxDirectory))
  }

  project {
    idea.project {
      (this as ExtensionAware)
      configure<ProjectSettings> {
        // TODO
      }
    }
  }
}

val clionVersion = properties["clionVersion"] as String
val userHomeDir = System.getProperty("user.home")!!
val clionInstallPath = "${project.projectDir}/build/clion/clion-$clionVersion"
val clionJarDir = "$clionInstallPath/lib/clion.jar"
val downloadURL = "https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz"
val buildSrcBuildDir = "${project.rootDir}/buildSrc/build/"
val sampleRepo = "https://github.com/duckietown/Software.git"
val samplePath = "${project.buildDir}/Software"

val defaultProjectPath = samplePath.let {
  if (File(it).isDirectory) it
  else it.apply { Grgit.clone(mapOf("dir" to this, "uri" to sampleRepo)) }
}

val projectPath = File(properties["roject"] as? String
  ?: System.getenv()["DUCKIETOWN_ROOT"] ?: defaultProjectPath).absolutePath!!

val isPluginDev = hasProperty("luginDev")
fun prop(name: String): String = extra.properties[name] as? String
  ?: error("Property `$name` is not defined in gradle.properties")

tasks {
  withType<PublishTask> {
    username(prop("publishUsername"))
    password(prop("publishPassword"))
    channels(prop("publishChannel"))
  }

  val downloadClion by creating(Download::class) {
    onlyIf { !file("$clionInstallPath.tar.gz").exists() }
    src(downloadURL)
    dest(file("$clionInstallPath.tar.gz"))
  }

  val unpackClion by creating(Copy::class) {
    onlyIf { !file(clionInstallPath).exists() }
    from(tarTree("build/clion/clion-$clionVersion.tar.gz"))
    into(file("${project.projectDir}/build/clion"))
    dependsOn(downloadClion)
  }

  val rosTask by withRosTask()

  withType<RunIdeTask> {
    if (!isPluginDev) dependsOn(unpackClion, rosTask, ":build_envs")

    args = listOf(if (isPluginDev) projectDir.absolutePath else projectPath)
  }

  val generateROSInterfaceLexer by creating(GenerateLexer::class) {
    source = "src/main/grammars/ROSInterface.flex"
    targetDir = "src/main/java/edu/umontreal/hatchery/rosinterface"
    targetClass = "ROSInterfaceLexer"
    purgeOldFiles = true
  }

  val generateROSInterfaceParser by creating(GenerateParser::class) {
    source = "src/main/grammars/ROSInterface.bnf"
    targetRoot = "src/main/java"
    pathToParser = "/edu/umontreal/hatchery/parser/ROSInterfaceParser.java"
    pathToPsiRoot = "/edu/umontreal/hatchery/psi"
    purgeOldFiles = true
  }

  withType<KotlinCompile> {
    dependsOn(generateROSInterfaceLexer, generateROSInterfaceParser)
    kotlinOptions.jvmTarget = JavaVersion.VERSION_1_8.toString()
    kotlinOptions.freeCompilerArgs += "-progressive"
  }
}


intellij {
  pluginName = "hatchery"
  version = clionVersion
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false
  if (!isPluginDev) alternativeIdePath = clionInstallPath

  setPlugins("name.kropp.intellij.makefile:1.5",   // Makefile support
    "org.intellij.plugins.markdown:183.4284.36",   // Markdown support
    "net.seesharpsoft.intellij.plugins.csv:1.9.1", // CSV file support
    "com.intellij.ideolog:183.0.7.0",              // Log file support
    "Pythonid:2018.3.183.4284.148",                 // Python   support
    "BashSupport:1.7.3",                           // [Ba]sh   support
    "Docker:183.4284.148",                          // Docker   support
    "PsiViewer:183.2153",                        // PSI view support
//      "IdeaVIM:0.49",
//      "AceJump:3.5.0",
    "yaml")                                        // YML file support
}

sourceSets["main"].compileClasspath += files(clionJarDir, buildSrcBuildDir)

dependencies {
  // gradle-intellij-plugin doesn't attach sources properly for Kotlin :(
  compileOnly(kotlin("stdlib-jdk8"))
  // Share ROS libraries for identifying the ROS home directory
  compile(fileTree(buildSrcBuildDir))
  compile(gradleApi())
  // Used for remote deployment over SCP
  compile("com.hierynomus:sshj:0.26.0")
  compile("com.jcraft:jzlib:1.1.3")
}

envs {
  bootstrapDirectory = File(buildDir, "pythons")
  envsDirectory = File(buildDir, "envs")

  conda("Miniconda2", "Miniconda2-latest", listOf("numpy", "pillow"))
// TODO: figure out how to setup conda inside the project structure
}

group = "edu.umontreal"
version = "0.2.1"