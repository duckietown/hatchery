package edu.umontreal.hatchery.rosinterface

import com.intellij.lexer.Lexer
import com.intellij.openapi.editor.DefaultLanguageHighlighterColors
import com.intellij.openapi.editor.HighlighterColors
import com.intellij.openapi.editor.colors.TextAttributesKey
import com.intellij.openapi.editor.colors.TextAttributesKey.createTextAttributesKey
import com.intellij.openapi.fileTypes.SyntaxHighlighterBase
import com.intellij.psi.TokenType
import com.intellij.psi.tree.IElementType
import edu.umontreal.hatchery.psi.ROSInterfaceTypes

class ROSInterfaceSyntaxHighlighter : SyntaxHighlighterBase() {
    val SEPARATOR = createTextAttributesKey("SIMPLE_SEPARATOR", DefaultLanguageHighlighterColors.OPERATION_SIGN)
    val TYPE = createTextAttributesKey("SIMPLE_KEY", DefaultLanguageHighlighterColors.KEYWORD)
    val KEY = createTextAttributesKey("SIMPLE_KEY", DefaultLanguageHighlighterColors.IDENTIFIER)
    val VALUE = createTextAttributesKey("SIMPLE_VALUE", DefaultLanguageHighlighterColors.STRING)
    val COMMENT = createTextAttributesKey("SIMPLE_COMMENT", DefaultLanguageHighlighterColors.LINE_COMMENT)
    val BAD_CHARACTER = createTextAttributesKey("SIMPLE_BAD_CHARACTER", HighlighterColors.BAD_CHARACTER)

    private val BAD_CHAR_KEYS = arrayOf(BAD_CHARACTER)
    private val SEPARATOR_KEYS = arrayOf(SEPARATOR)
    private val TYPE_KEYS = arrayOf(TYPE)
    private val KEY_KEYS = arrayOf(KEY)
    private val VALUE_KEYS = arrayOf(VALUE)
    private val COMMENT_KEYS = arrayOf(COMMENT)
    private val EMPTY_KEYS = arrayOf<TextAttributesKey>()

    override fun getHighlightingLexer() = ROSInterfaceLexerAdapter()

    override fun getTokenHighlights(tokenType: IElementType) =
            when (tokenType) {
                ROSInterfaceTypes.TYPE -> TYPE_KEYS
                ROSInterfaceTypes.SEPARATOR -> SEPARATOR_KEYS
                ROSInterfaceTypes.KEY -> KEY_KEYS
                ROSInterfaceTypes.VALUE -> VALUE_KEYS
                ROSInterfaceTypes.COMMENT -> COMMENT_KEYS
                TokenType.BAD_CHARACTER -> BAD_CHAR_KEYS
                else -> EMPTY_KEYS
            }
}