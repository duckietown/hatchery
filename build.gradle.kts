import de.undercouch.gradle.tasks.download.Download
import org.ajoberstar.grgit.Grgit
import org.apache.tools.ant.taskdefs.ExecTask
import org.gradle.api.tasks.JavaExec
import org.gradle.kotlin.dsl.getValue
import org.gradle.kotlin.dsl.getting
import org.gradle.kotlin.dsl.kotlin
import org.gradle.kotlin.dsl.version
import org.jetbrains.intellij.tasks.PublishTask
import org.gradle.language.base.internal.plugins.CleanRule
import org.jetbrains.grammarkit.GrammarKit
import org.jetbrains.intellij.tasks.RunIdeTask
import org.jetbrains.kotlin.gradle.tasks.KotlinCompile
import org.jetbrains.grammarkit.GrammarKitPluginExtension
import org.jetbrains.grammarkit.tasks.*
import org.jetbrains.kotlin.backend.common.onlyIf
import org.jetbrains.kotlin.cli.jvm.main

plugins {
  idea apply true
  kotlin("jvm") version "1.2.71" apply true
  // TODO: https://github.com/JetBrains/gradle-python-envs#usage
  id("com.jetbrains.python.envs") version "0.0.25"
  id("org.jetbrains.intellij") version "0.3.11" apply true
  id("de.undercouch.download") version "3.4.3" apply true
  id("org.jetbrains.grammarkit") version "2018.2" apply true
  id("com.google.cloud.tools.jib") version "0.9.11"
  id("org.ajoberstar.grgit") version "3.0.0-rc.2" apply true
}

// TODO: Maybe these should go in settings.gradle.kts?
val rosDistro = "kinetic"
val clionVersion = "2018.2.4"
val userHomeDir = System.getProperty("user.home")!!

val installPath = "${project.projectDir}/build/clion/clion-$clionVersion"
val downloadURL = "https://download.jetbrains.com/cpp/CLion-$clionVersion.tar.gz"
val sampleRepo = "https://github.com/duckietown/Software.git"
val samplePath = "${project.buildDir}/Software"

val defaultProjectPath = samplePath.let { if (File(it).isDirectory) it else sampleRepo }
var projectPath = properties["roject"] as? String ?: System.getenv()["DUCKIETOWN_ROOT"] ?: defaultProjectPath
val isPluginDev = hasProperty("luginDev")
fun cloneProject(url: String) = samplePath.apply { Grgit.clone(mapOf("dir" to this, "uri" to url)) }
projectPath = projectPath.let { if (it.startsWith("http")) cloneProject(it) else it }

val rosProjectRoot = File(projectPath)/*.walkTopDown()
    .filter { it.name == "CMakeLists.txt" }
    // Since FileTreeWalk does not support BFS...
    .sortedBy { it.absolutePath.length }
    .firstOrNull()
    ?.parentFile*/

val LOG = project.logger

val rosPython = "/opt/ros/$rosDistro/lib/python2.7/dist-packages"

fun prop(name: String): String =
    extra.properties[name] as? String
        ?: error("Property `$name` is not defined in gradle.properties")

tasks {
  withType<PublishTask> {
    username(prop("publishUsername"))
    password(prop("publishPassword"))
    channels(prop("publishChannel"))
  }

  val downloadClion by creating(Download::class) {
    onlyIf { !file("$installPath.tar.gz").exists() }
    src(downloadURL)
    dest(file("$installPath.tar.gz"))
  }

  val unpackClion by creating(Copy::class) {
    onlyIf { !file(installPath).exists() }
    from(tarTree("build/clion/clion-$clionVersion.tar.gz"))
    into(file("${project.projectDir}/build/clion"))
    dependsOn(downloadClion)
  }

  val setupRosEnv by creating(Exec::class) {
    try {
      var rosSetupFile = File("/opt/ros/$rosDistro/setup.bash")
      if (rosSetupFile.exists())
        commandLine("bash", rosSetupFile)
      else if (File("/opt/ros/").isDirectory) {
        rosSetupFile = File("/opt/ros/").walkTopDown().first { it.name == "setup.bash" }
        executable = "source $rosSetupFile"
        LOG.warn("Unable to find default ROS distro ($rosDistro), using ${rosSetupFile.parentFile.name} instead")
      } else LOG.error("Unable to detect a usable setup.bash file in /opt/ros!")

      val srcRoot = rosProjectRoot.walkTopDown().first { it.isDirectory && it.name == "catkin_ws" }.absolutePath

      isIgnoreExitValue = true
      commandLine("bash", rosSetupFile, "catkin_make", "-C", srcRoot)
      val rosDevScript = File(srcRoot).resolveSibling("devel/setup.bash")
      rosDevScript.setExecutable(true)
      commandLine("bash", rosDevScript.absolutePath)
    } catch (e: Exception) {
      executable = "echo"
      LOG.error("Could not configure ROS environment", e)
    }
  }

  withType<RunIdeTask> {
    if(!isPluginDev) dependsOn(unpackClion, setupRosEnv)

    // Try to set Python SDK default to ROS Python...
    val pythonPath = System.getenv()["PYTHONPATH"] ?: ""
    environment = mutableMapOf("PYTHONPATH" to "$rosPython:$pythonPath")
        .apply { putAll(System.getenv()) } as Map<String, Any>
    LOG.info("Python path: " + environment["PYTHONPATH"])
    LOG.info("Project root directory: $rosProjectRoot")

    args = listOf(if(isPluginDev) projectDir.absolutePath else rosProjectRoot.absolutePath)
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
    kotlinOptions.jvmTarget = "1.8"
  }
}

sourceSets["main"].compileClasspath += files("$installPath/lib/clion.jar")

intellij {
  pluginName = "hatchery"
  version = clionVersion
  updateSinceUntilBuild = false
  if (hasProperty("roject")) downloadSources = false
  if(!isPluginDev) alternativeIdePath = "build/clion/clion-$clionVersion"

  setPlugins("name.kropp.intellij.makefile:1.3",     // Makefile support
      "org.intellij.plugins.markdown:182.2371",      // Markdown support
      "net.seesharpsoft.intellij.plugins.csv:1.8.0", // CSV file support
      "com.intellij.ideolog:182.0.7.0",              // Log file support
      "Pythonid:2018.2.182.3684.101",                // Python   support
      "BashSupport:1.6.13.182",                      // [Ba]sh   support
      "Docker:182.3684.90",                          // Docker   support
      "PsiViewer:182.2757.2",                        // PSI view support
//      "IdeaVIM:0.49",
//      "AceJump:3.5.0",
      "yaml")                                        // YML file support
}

envs {
  bootstrapDirectory = File(buildDir, "pythons")
  envsDirectory = File(buildDir, "envs")

  conda("Miniconda2", "Miniconda2-latest", listOf("numpy", "pillow"))
  // TODO: figure out how to setup conda inside the project structure
}

group = "edu.umontreal"
version = "0.2.1"
