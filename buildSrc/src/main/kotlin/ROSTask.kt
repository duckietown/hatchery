import org.gradle.api.*
import org.gradle.api.tasks.*
import org.gradle.kotlin.dsl.*

open class ROSTask : DefaultTask() {
  init {
    group = "My"
    description = "Prints a description of ${project.name}."
  }

  @TaskAction
  fun run() {
    println("I'm ${project.name}")
  }
}

/**
 * Declares a [ROSTask] named `hello`.
 */
fun Project.withRosTask() = tasks.register("hello", ROSTask::class)
