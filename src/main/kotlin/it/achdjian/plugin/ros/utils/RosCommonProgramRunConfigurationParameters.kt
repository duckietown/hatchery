package it.achdjian.plugin.ros.utils

import com.intellij.execution.CommonProgramRunConfigurationParameters
import com.intellij.openapi.project.Project
import com.intellij.util.execution.ParametersListUtil.DEFAULT_LINE_PARSER
import org.jdom.Element

class RosCommonProgramRunConfigurationParameters(private val prj: Project) : CommonProgramRunConfigurationParameters {

    companion object {
        const val ARGUMENTS_TAG = "arguments"
        const val WORKING_DIR = "workingDir"
        const val ENVIRONMENT = "environment"
        const val PASS_PARENT_ENV = "passParentEnv"
    }

    private var arguments: String? = null
    private var workingDir: String? = null
    private var env: MutableMap<String, String> = HashMap()
    private var setPassParentEnvs: Boolean = false


    override fun getWorkingDirectory(): String? = workingDir

    override fun getEnvs(): MutableMap<String, String> = env

    override fun setWorkingDirectory(value: String?) {
        workingDir = value
    }

    override fun setEnvs(envs: MutableMap<String, String>) {
        env = envs
    }

    override fun isPassParentEnvs() = setPassParentEnvs

    override fun setPassParentEnvs(passParentEnvs: Boolean) {
        setPassParentEnvs = passParentEnvs
    }

    override fun setProgramParameters(value: String?) {
        arguments = value
    }

    override fun getProgramParameters() = arguments

    fun getProgramParametersList() = DEFAULT_LINE_PARSER.`fun`(arguments)

    override fun getProject() = prj

    fun readExternal(parentElement: Element) {
        parentElement.getAttributeValue(ARGUMENTS_TAG)?.let { arguments = it }
        parentElement.getAttributeValue(WORKING_DIR)?.let { workingDir = it }
        parentElement.getChild(ENVIRONMENT)?.let { envElement ->
            env = HashMap(envElement
                    .content
                    .filter { it is Element }
                    .map { it as Element }
                    .map { it.name to it.text }
                    .toMap())
        }
        parentElement.getAttributeValue(PASS_PARENT_ENV)?.let { setPassParentEnvs = it.toBoolean() }
    }

    fun writeExternal(parentElement: Element) {
        arguments?.let { parentElement.setAttribute(ARGUMENTS_TAG, it) }
        workingDir?.let { parentElement.setAttribute(WORKING_DIR, it) }
        val contentEnv = Element(ENVIRONMENT)
        contentEnv.addContent(env.map { entry -> createDomEnvironmentEntry(entry) })
        parentElement.setAttribute(PASS_PARENT_ENV, setPassParentEnvs.toString())

    }


    private fun createDomEnvironmentEntry(entry: Map.Entry<String, String>): Element {
        val element = Element(entry.key)
        element.text = entry.value
        return element
    }
}