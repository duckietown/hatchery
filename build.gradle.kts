import org.ajoberstar.grgit.Grgit

val kotlinVersion = properties["kotlinVersion"] as String

plugins {
  idea apply true
  kotlin("jvm") version "1.7.10"
  // TODO: https://github.com/JetBrains/gradle-python-envs#usage
  id("com.jetbrains.python.envs") version "0.0.31"
  id("org.jetbrains.intellij") version "1.7.0"
  id("org.jetbrains.grammarkit") version "2021.2.2"
  id("org.ajoberstar.grgit") version "5.0.0"
  id("com.github.ben-manes.versions") version "0.42.0"
}

idea {
  module {
    isDownloadJavadoc = true
    isDownloadSources = true
    generatedSourceDirs.add(file("src/main/java"))
//    excludeDirs.add(file(intellij.sandboxDirectory))
  }
}

val userHomeDir = System.getProperty("user.home")!!
val sampleRepo = "https://github.com/duckietown/dt-core.git"
val samplePath = "${project.buildDir}/Software"

val defaultProjectPath = samplePath.let {
  if (File(it).isDirectory) it
  else it.apply {
    logger.info("Cloning $sampleRepo to $samplePath...")
    Grgit.clone(mapOf("dir" to this, "uri" to sampleRepo))
  }
}

val projectPath = File(properties["roject"] as? String
  ?: System.getenv()["DUCKIETOWN_ROOT"] ?: defaultProjectPath).absolutePath

val isCIBuild = hasProperty("CI")
val isPluginDev = hasProperty("luginDev")
fun prop(name: String): String = extra.properties[name] as? String
  ?: error("Property `$name` is not defined in gradle.properties")

tasks {
  publishPlugin {
    token.set(project.findProperty("jbr.token") as String? ?: System.getenv("JBR_TOKEN"))
  }

  patchPluginXml {
    sinceBuild.set("192.*")
    changeNotes.set("Fixes an error parsing .msg/.srv files and run configuration issue on older platform versions.")
  }

  named("buildPlugin") { dependsOn("test") }

  withType<Zip> {
    archiveFileName.set("hatchery.zip")
  }

  runIde {
//    dependsOn("test")

    if (!isPluginDev && !isCIBuild) dependsOn(":build_envs")

    args = listOf(if (isPluginDev) projectDir.absolutePath else projectPath)
  }

  findByName("buildSearchableOptions")?.enabled = false

  generateLexer {
    source.set("src/main/grammars/ROSInterface.flex")
    targetDir.set("src/main/java/org/duckietown/hatchery/rosinterface")
    targetClass.set("ROSInterfaceLexer")
    purgeOldFiles.set(true)
  }

  generateParser {
    source.set("src/main/grammars/ROSInterface.bnf")
    targetRoot.set("src/main/java")
    pathToParser.set("/org/duckietown/hatchery/parser/ROSInterfaceParser.java")
    pathToPsiRoot.set("/org/duckietown/hatchery/psi")
    purgeOldFiles.set(true)
  }

  compileKotlin {
    dependsOn(generateLexer, generateParser)
    kotlinOptions {
      jvmTarget = JavaVersion.VERSION_1_8.toString()
      languageVersion = kotlinVersion.substringBeforeLast('.')
      apiVersion = languageVersion
      freeCompilerArgs = listOf("-progressive")
    }
  }

  // Use unversioned filename for stable URL of CI build artifact
  register("copyPlugin", Copy::class) {
    from("${buildDir}/libs/hatchery.zip")

    into("${project.gradle.gradleUserHomeDir}/../.CLion2020.1/config/plugins/hatchery/lib")
  }

  register("Exec clion debug suspend", Exec::class){
      commandLine("/opt/clion/bin/clion-suspend.sh")
  }
}

intellij {
  type.set("CL")
  version.set("2020.2")
//  version.set("2022.1.3") // TODO: migrate to new API


  pluginName.set("hatchery")
  updateSinceUntilBuild.set(false)
  if (hasProperty("roject")) downloadSources.set(false)

  plugins.set(listOf(
    //"com.intellij.ideolog:193.0.15.0",  // Log file support
    //"BashSupport:1.7.13.192",           // [Ba]sh   support
    //"Docker:193.4386.10",               // Docker   support
    //"PsiViewer:201-SNAPSHOT",           // PSI view support
    "clion-embedded",
    "IntelliLang",
    "yaml"
  ))
}

repositories {
  maven("https://raw.githubusercontent.com/rosjava/rosjava_mvn_repo/master")
}

dependencies {
  // gradle-intellij-plugin doesn't attach sources properly for Kotlin :(
  compileOnly(kotlin("stdlib-jdk8"))
  // Share ROS libraries for identifying the ROS home directory
  // Used for remote deployment over SCP
//  compile("com.hierynomus:sshj:0.27.0")
//  compile("com.jcraft:jzlib:1.1.3")

  // Useful ROS Dependencies
  testImplementation("org.ros.rosjava_core:rosjava:0.3+")
  testImplementation("org.ros.rosjava_messages:std_msgs:0.5+")
  testImplementation("org.ros.rosjava_bootstrap:message_generation:0.3+")
}

envs {
  bootstrapDirectory = File(buildDir, "pythons")
  envsDirectory = File(buildDir, "envs")

//  conda("Miniconda2", "Miniconda2-latest", listOf("numpy", "pillow"))
// TODO: figure out how to setup conda inside the project structure
}

group = "org.duckietown"
version = "0.3.4"
