import de.undercouch.gradle.tasks.download.Download
import edu.umontreal.hatchery.withRosTask
import org.ajoberstar.grgit.Grgit
import org.jetbrains.grammarkit.tasks.GenerateLexer
import org.jetbrains.grammarkit.tasks.GenerateParser
import org.jetbrains.intellij.tasks.PublishTask
import org.jetbrains.intellij.tasks.RunIdeTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile

val kotlinVersion = properties["kotlinVersion"] as String

buildscript {
  repositories {
    maven("https://dl.bintray.com/kotlin/kotlin-eap")
    maven("https://raw.githubusercontent.com/rosjava/rosjava_mvn_repo/master")
  }

  dependencies {
    val kotlinVersion = properties["kotlinVersion"] as String
    classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlinVersion")
  }
}

plugins {
  idea apply true
  kotlin("jvm")
  id("ros-java") version "0.3.0" apply true
  // TODO: https://github.com/JetBrains/gradle-python-envs#usage
//  id("org.ros2.tools.gradle") version "0.7.0" apply true
  id("com.jetbrains.python.envs") version "0.0.30" apply true
  id("org.jetbrains.intellij") version "0.4.5" apply true
  id("de.undercouch.download") version "3.4.3" apply true
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

//  project {
//    idea.project {
//      (this as ExtensionAware)
//      configure<ProjectSettings> {
//        runConfigurations {
//          create<org.jetbrains.gradle.ext.Application>("Run Hatchery") {
//            beforeRun.create<GradleTask>("runIde") {
//              task = tasks.getByPath("runIde")
//            }
//          }
//        }
//      }
//    }
//  }
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

  val downloadClion by creating(Download::class) {
    onlyIf { !file("$clionInstallPath.tar.gz").exists() }
    src(downloadURL)
    dest(file("$clionInstallPath.tar.gz"))
  }

  val unpackClion by creating(Copy::class) {
    onlyIf { file("$clionInstallPath.tar.gz").exists() && !file(clionInstallPath).exists() }
    from(tarTree("$clionInstallPath.tar.gz"))
    into(file("$clionInstallPath/.."))
    dependsOn(downloadClion)
  }

  val rosTask by withRosTask()

  named("buildPlugin") { dependsOn("test") }

  withType<RunIdeTask> {
    dependsOn("test")

    if (!isPluginDev && !isCIBuild) dependsOn(unpackClion, rosTask, ":build_envs")

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
//  type = "CL" // <-- when this line is added, the build fails
//  version = "CL-$clionVersion-EAP-SNAPSHOT"
  pluginName = "hatchery"
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false
  if (!isPluginDev) alternativeIdePath = "$clionInstallPath/"

  setPlugins("name.kropp.intellij.makefile:1.6",   // Makefile support
    "org.intellij.plugins.markdown:191.5109.14",   // Markdown support
    "net.seesharpsoft.intellij.plugins.csv:2.2.1", // CSV file support
    "com.intellij.ideolog:191.0.7.0",              // Log file support
    "Pythonid:2019.1.191.4212.41",                 // Python   support
    "BashSupport:1.7.5@eap",                       // [Ba]sh   support
    "Docker:191.5701.16",                          // Docker   support
    "PsiViewer:191.4212",                          // PSI view support
    "yaml")
}

sourceSets["main"].compileClasspath += files(clionJarDir, buildSrcBuildDir)

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
}

envs {
  bootstrapDirectory =  File(buildDir, "pythons")
  envsDirectory = File(buildDir, "envs")

  conda("Miniconda2", "Miniconda2-latest", listOf("numpy", "pillow"))
// TODO: figure out how to setup conda inside the project structure
}

group = "edu.umontreal"
version = "0.3"