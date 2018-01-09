package edu.umontreal.hatchery.psi.impl

import com.intellij.lang.ASTNode
import com.intellij.navigation.ItemPresentation
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFile
import edu.umontreal.hatchery.psi.*

import javax.swing.*

object SimplePsiImplUtil {
    fun getKey(element: SimpleProperty): String? {
        val keyNode = element.getNode().findChildByType(SimpleTypes.KEY)
        return if (keyNode != null) {
            // IMPORTANT: Convert embedded escaped spaces to simple spaces
            keyNode!!.getText().replace("\\\\ ".toRegex(), " ")
        } else {
            null
        }
    }

    fun getValue(element: SimpleProperty): String? {
        val valueNode = element.getNode().findChildByType(SimpleTypes.VALUE)
        return if (valueNode != null) {
            valueNode!!.getText()
        } else {
            null
        }
    }

    fun getName(element: SimpleProperty): String? {
        return getKey(element)
    }

    fun setName(element: SimpleProperty, newName: String): PsiElement {
        val keyNode = element.getNode().findChildByType(SimpleTypes.KEY)
        if (keyNode != null) {
            val property = SimpleElementFactory.createProperty(element.getProject(), newName)
            val newKeyNode = property.getFirstChild().getNode()
            element.getNode().replaceChild(keyNode, newKeyNode)
        }
        return element
    }

    fun getNameIdentifier(element: SimpleProperty): PsiElement? {
        val keyNode = element.getNode().findChildByType(SimpleTypes.KEY)
        return if (keyNode != null) {
            keyNode!!.getPsi()
        } else {
            null
        }
    }

    fun getPresentation(element: SimpleProperty): ItemPresentation {
        return object : ItemPresentation {
            override fun getPresentableText(): String? {
                return element.name
//                return element.getKey()
            }

            override fun getLocationString(): String? {
                val containingFile = element.getContainingFile()
                return if (containingFile == null) null else containingFile!!.getName()
            }

            override fun getIcon(unused: Boolean): Icon? {
                return null
            }
        }
    }
}
