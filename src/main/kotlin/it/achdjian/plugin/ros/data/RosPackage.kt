package it.achdjian.plugin.ros.data

import com.intellij.openapi.diagnostic.Logger
import it.achdjian.plugin.ros.utils.catkinFindLibexec
import org.w3c.dom.Document
import org.w3c.dom.NodeList
import java.nio.file.Files
import java.nio.file.Path
import javax.xml.parsers.DocumentBuilderFactory
import javax.xml.xpath.XPath
import javax.xml.xpath.XPathConstants
import javax.xml.xpath.XPathFactory

class RosPackage(val path: Path, val env: Map<String, String>) {
    companion object {
        val log = Logger.getInstance(RosPackage::class.java)
    }


    val name: String
    val version: String
    val description: String
    private var rosNodes: List<RosNode>? = null

    init {
        log.trace("package at $path, with ${env.size} environment")
        val packageFile = path.resolve("package.xml")
        val documentBuilder = DocumentBuilderFactory.newInstance()
        val builder = documentBuilder.newDocumentBuilder()
        val doc = builder.parse(packageFile.toFile())

        val xpFactory = XPathFactory.newInstance()
        val xPath = xpFactory.newXPath()

        val xpathName = "/package/name"

        name = getNodeValue(xPath, "/package/name", doc)
        version = getNodeValue(xPath, "/package/version", doc)
        description = getNodeValue(xPath, "/package/description", doc)

    }

    fun getNodes(): List<RosNode> = rosNodes ?: searchRosNodes()


    private fun searchRosNodes(): List<RosNode> {
        val foundNodes = ArrayList<RosNode>()
        catkinFindLibexec(name, env).forEach { libExecPath ->
            Files.walk(libExecPath).filter { !Files.isDirectory(it) }.filter { Files.isExecutable(it) }.map { RosNode(it) }.forEach { foundNodes.add(it) }
        }
        this.rosNodes = foundNodes
        return foundNodes
    }

    private fun getNodeValue(xPath: XPath, xpath: String, doc: Document): String {
        val nodes = xPath.evaluate(xpath, doc, XPathConstants.NODESET) as NodeList
        return if (nodes.length > 0)
            nodes.item(0).textContent.trim()
        else
            ""
    }
}