import edu.umontreal.hatchery.withRosTask
import org.ajoberstar.grgit.Grgit
import org.jetbrains.grammarkit.tasks.GenerateLexer
import org.jetbrains.grammarkit.tasks.GenerateParser
import org.jetbrains.intellij.tasks.PublishTask
import org.jetbrains.intellij.tasks.RunIdeTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val kotlinVersion = properties["kotlinVersion"] as String

buildscript {
  dependencies {
    val kotlinVersion = properties["kotlinVersion"] as String
    classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlinVersion")
  }
}

plugins {
  idea apply true
  kotlin("jvm")
  // TODO: https://github.com/JetBrains/gradle-python-envs#usage
  id("com.jetbrains.python.envs") version "0.0.30" apply true
  id("org.jetbrains.intellij") version "0.4.6" apply true
  id("org.jetbrains.grammarkit") version "2018.3" apply true
  id("org.ajoberstar.grgit") version "3.0.0" apply true
//  id("org.jetbrains.gradle.plugin.idea-ext") version "0.3" apply true
}

idea {
  module {
    isDownloadSources = true
    generatedSourceDirs.add(file("src/main/java"))
    excludeDirs.add(file(intellij.sandboxDirectory))
  }
}

val userHomeDir = System.getProperty("user.home")!!
val buildSrcBuildDir = "${project.rootDir}/buildSrc/build/libs/buildSrc.jar"
val sampleRepo = "https://github.com/duckietown/Software.git"
val samplePath = "${project.buildDir}/Software"

val defaultProjectPath = samplePath.let {
  if (File(it).isDirectory) it
  else it.apply {
    logger.info("Cloning $sampleRepo to $samplePath...")
    Grgit.clone(mapOf("dir" to this, "uri" to sampleRepo))
  }
}

val projectPath = File(properties["roject"] as? String
  ?: System.getenv()["DUCKIETOWN_ROOT"] ?: defaultProjectPath).absolutePath!!

val isCIBuild = hasProperty("CI")
val isPluginDev = hasProperty("luginDev")
fun prop(name: String): String = extra.properties[name] as? String
  ?: error("Property `$name` is not defined in gradle.properties")

tasks {
  withType<PublishTask> {
    username(prop("publishUsername"))
    password(prop("publishPassword"))
    channels(prop("publishChannel"))
  }

  val rosTask by withRosTask()

  named("buildPlugin") { dependsOn("test") }
  
  withType<Zip> {
    archiveFileName.set("hatchery.zip")
  }

  withType<RunIdeTask> {
    dependsOn("test")

    if (!isPluginDev && !isCIBuild) dependsOn(rosTask, ":build_envs")

    args = listOf(if (isPluginDev) projectDir.absolutePath else projectPath)
  }

  findByName("buildSearchableOptions")?.enabled = false

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
    kotlinOptions {
      jvmTarget = JavaVersion.VERSION_1_8.toString()
      languageVersion = kotlinVersion.substringBeforeLast('.')
      apiVersion = languageVersion
      freeCompilerArgs = listOf("-progressive")
    }
  }
}

intellij {
  type = "CL"
  version = "CL-LATEST-EAP-SNAPSHOT"
  pluginName = "hatchery"
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false

  setPlugins("name.kropp.intellij.makefile:1.6.1", // Makefile support
    "org.intellij.plugins.markdown:191.5849.16",   // Markdown support
    "net.seesharpsoft.intellij.plugins.csv:2.3.0", // CSV file support
    "com.intellij.ideolog:191.0.7.0",              // Log file support
    "BashSupport:1.7.6",                           // [Ba]sh   support
    "Docker:191.6183.53",                          // Docker   support
    "PsiViewer:191.4212",                          // PSI view support
    "IntelliLang",
    "yaml")
}

sourceSets["main"].compileClasspath += files(buildSrcBuildDir)

repositories {
  jcenter()
  maven("https://raw.githubusercontent.com/rosjava/rosjava_mvn_repo/master")
}

dependencies {
  // gradle-intellij-plugin doesn't attach sources properly for Kotlin :(
  compileOnly(kotlin("stdlib-jdk8"))
  // Share ROS libraries for identifying the ROS home directory
  compile(fileTree(buildSrcBuildDir))
  compileOnly(gradleApi())
  // Used for remote deployment over SCP
  compile("com.hierynomus:sshj:0.27.0")
  compile("com.jcraft:jzlib:1.1.3")

  // Useful ROS Dependencies
  compile("org.ros.rosjava_core:rosjava:[0.3,)")
  compile("org.ros.rosjava_messages:std_msgs:[0.5,)")
  compile("org.ros.rosjava_bootstrap:message_generation:[0.3,)")

  // Python
  compile("org.python:jython-standalone:2.7.1")
}

envs {
  bootstrapDirectory =  File(buildDir, "pythons")
  envsDirectory = File(buildDir, "envs")

  conda("Miniconda2", "Miniconda2-latest", listOf("numpy", "pillow"))
// TODO: figure out how to setup conda inside the project structure
}

group = "edu.umontreal"
version = "0.3"