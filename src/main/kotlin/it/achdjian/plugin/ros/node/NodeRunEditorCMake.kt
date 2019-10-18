package it.achdjian.plugin.ros.node

import com.intellij.execution.ui.CommonProgramParametersPanel
import com.intellij.openapi.diagnostic.Logger
import com.intellij.openapi.project.Project
import com.intellij.openapi.ui.ComboBox
import com.intellij.ui.components.JBTextField
import com.intellij.ui.components.fields.IntegerField
import com.intellij.ui.layout.panel
import com.jetbrains.cidr.cpp.execution.CMakeAppRunConfiguration
import com.jetbrains.cidr.cpp.execution.CMakeAppRunConfigurationSettingsEditor
import com.jetbrains.cidr.cpp.execution.CMakeBuildConfigurationHelper
import com.jetbrains.cidr.execution.BuildTargetAndConfigurationData
import com.jetbrains.cidr.execution.BuildTargetData
import com.jetbrains.cidr.execution.ExecutableData
import it.achdjian.plugin.ros.data.RosNode
import it.achdjian.plugin.ros.data.RosPackage
import it.achdjian.plugin.ros.ui.RosNodeListCellRenderer
import it.achdjian.plugin.ros.ui.RosPackageListCellRenderer
import it.achdjian.plugin.ros.utils.getPackages

class NodeRunEditorCMake(val project: Project, helper: CMakeBuildConfigurationHelper) : CMakeAppRunConfigurationSettingsEditor(project, helper) {
    companion object {
        private val LOG = Logger.getInstance(NodeRunEditorCMake::class.java)

    }

    private val comboPackages = ComboBox<RosPackage>()
    private val comboNodes = ComboBox<RosNode>()
    private val programParametersPanel = CommonProgramParametersPanel()
    private val packages = getPackages(project)
    private val rosMasterAddr = JBTextField("127.0.0.1")
    private val rosMasterPort = IntegerField("11311", 0, 65535)
    private val allTarget = helper.createBuildAllVirtualTarget()


    init {
        comboPackages.renderer = RosPackageListCellRenderer()
        comboNodes.renderer = RosNodeListCellRenderer()
        packages.forEach { comboPackages.addItem(it) }
        comboPackages.addActionListener { actionEvent ->
            LOG.trace("ActionEvent: ${actionEvent}")
            val selected = comboPackages.selectedItem as RosPackage
            LOG.trace("selected: ${selected.name}")
            comboNodes.removeAllItems()
            selected.getNodes().forEach { comboNodes.addItem(it) }
        }


        val selected = comboPackages.selectedItem as RosPackage
        selected.getNodes().forEach { comboNodes.addItem(it) }
    }

    override fun resetEditorFrom(cmakeConfiguration: CMakeAppRunConfiguration) {
        val configuration = cmakeConfiguration as NodeConfiguration

        packages.firstOrNull { it.name == configuration.rosPackageName }?.let {
            comboPackages.selectedItem = it
            comboNodes.selectedItem = it.getNodes().firstOrNull { it.name == configuration.rosNodeName }
        }
        rosMasterAddr.text = configuration.rosMasterAddr
        rosMasterPort.value = configuration.rosMasterPort
        programParametersPanel.programParametersComponent.component.text = configuration.workingDirectory
        programParametersPanel.programParametersComponent.component.text = configuration.programParameters
        LOG.info("set with configuration: $configuration")
    }

    override fun applyEditorTo(cmakeConfiguration: CMakeAppRunConfiguration) {
        val configuration = cmakeConfiguration as NodeConfiguration
        configuration.rosMasterAddr = rosMasterAddr.text
        configuration.rosMasterPort = rosMasterPort.value

        comboPackages.selectedItem?.let {
            configuration.rosPackageName = (it as RosPackage).name
            configuration.envs = it.env
            rosMasterAddr.text?.let {
                if (it.isNotEmpty()) {
                    configuration.envs["ROS_MASTER_URI"] = "http://$it:${rosMasterPort.text}"
                }
            }
        }
        comboNodes.selectedItem?.let {
            configuration.rosNodeName = (it as RosNode).name
            configuration.executableData = ExecutableData(it.path.toString())

        }

        configuration.workingDirectory = programParametersPanel.programParametersComponent.component.text
        configuration.programParameters = programParametersPanel.programParametersComponent.component.text

        val targetData = BuildTargetData(allTarget)
        configuration.targetAndConfigurationData = BuildTargetAndConfigurationData(targetData,"Release")
        this.syncBuildAndExecute(cmakeConfiguration, targetData)
        configuration.setExplicitBuildTargetName("all")
    }

    override fun createEditor() = panel {
        row("Package") {
            comboPackages(grow)
        }

        row("Node") {
            comboNodes(grow)
        }
        titledRow("ROS MASTER") {
            row("ROS MASTER address") {
                rosMasterAddr(grow)
            }
            row("ROS MASTER port") {
                rosMasterPort(grow)
            }
        }
        row {
            programParametersPanel(grow)
        }

    }

}