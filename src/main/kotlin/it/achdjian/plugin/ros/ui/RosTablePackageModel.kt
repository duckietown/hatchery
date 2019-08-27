package it.achdjian.plugin.ros.ui

import com.intellij.openapi.diagnostic.Logger
import it.achdjian.plugin.ros.data.RosVersionImpl
import javax.swing.event.TableModelEvent
import javax.swing.event.TableModelListener
import javax.swing.table.TableModel

class RosTablePackageModel : TableModel {
    companion object {
        private val LOG = Logger.getInstance(RosTablePackageModel::class.java)
    }

    private var modelListener = HashSet<TableModelListener>()
    private var rosVersion: RosVersionImpl? = null

    fun updateVersions(newRosVersion: RosVersionImpl) {
        LOG.trace("Update version: ${newRosVersion.name}")
        rosVersion = newRosVersion
        newRosVersion.searchPackages()
        LOG.trace("Found ${newRosVersion.packages.size}")
        modelListener.forEach {
            LOG.trace("Notify table change")
            it.tableChanged(TableModelEvent(this))
        }
    }

    override fun addTableModelListener(listener: TableModelListener) {
        modelListener.add(listener)
    }

    override fun removeTableModelListener(listener: TableModelListener) {
        modelListener.remove(listener)
    }


    override fun getRowCount(): Int {
        return rosVersion?.packages?.size ?: 0
    }

    override fun getColumnName(colId: Int): String {
        val name = when (colId) {
            0 -> "Name"
            1 -> "Version"
            2 -> "Description"
            else -> {
                ""
            }
        }
        return name
    }

    override fun isCellEditable(p0: Int, p1: Int) = false

    override fun getColumnClass(p0: Int): Class<*> = String::class.java

    override fun setValueAt(p0: Any?, p1: Int, p2: Int) {
    }

    override fun getColumnCount() = 3

    override fun getValueAt(rowIndex: Int, colIndex: Int) = rosVersion?.let {
        val packages = it.packages
        if (packages.size >= rowIndex) {
            when (colIndex) {
                0 -> packages[rowIndex].name
                1 -> packages[rowIndex].version
                2 -> packages[rowIndex].description
                else -> {
                    ""
                }
            }
        } else {
            ""
        }
    } ?: ""
}
