package org.duckietown.hatchery.rosinterface

import com.intellij.lang.*
import com.intellij.lang.ParserDefinition.SpaceRequirements
import com.intellij.openapi.project.Project
import com.intellij.psi.*
import com.intellij.psi.tree.*
import org.duckietown.hatchery.psi.*

object RosInterfaceParserDefinition : ParserDefinition {
  val WHITE_SPACES = TokenSet.create(TokenType.WHITE_SPACE)
  val COMMENTS = TokenSet.create(RosInterfaceTypes.COMMENT)
  val FILE = IFileElementType(RosInterfaceLanguage)

  override fun createLexer(project: Project) = RosInterfaceLexerAdapter()

  override fun getWhitespaceTokens(): TokenSet = WHITE_SPACES

  override fun getCommentTokens(): TokenSet = COMMENTS

  override fun getStringLiteralElements(): TokenSet = TokenSet.EMPTY

  override fun createParser(project: Project) = RosInterfaceParser()

  override fun getFileNodeType(): IFileElementType = FILE

  override fun createFile(viewProvider: FileViewProvider): PsiFile = RosInterfaceFile(viewProvider)

  override fun spaceExistenceTypeBetweenTokens(left: ASTNode, right: ASTNode) = SpaceRequirements.MAY

  override fun createElement(node: ASTNode): PsiElement = RosInterfaceTypes.Factory.createElement(node)
}