plugins {
  kotlin("jvm") version "1.3.0-rc-190"
  `kotlin-dsl`
}

buildscript {
  dependencies.classpath("org.jetbrains.kotlin:kotlin-gradle-plugin:1.2.60")
  repositories.maven("https://dl.bintray.com/kotlin/kotlin-eap")
}

repositories {
  jcenter()
}