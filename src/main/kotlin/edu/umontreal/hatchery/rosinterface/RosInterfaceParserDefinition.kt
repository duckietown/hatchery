package edu.umontreal.hatchery.rosinterface

import com.intellij.lang.ASTNode
import com.intellij.lang.ParserDefinition
import com.intellij.lang.ParserDefinition.SpaceRequirements
import com.intellij.openapi.project.Project
import com.intellij.psi.FileViewProvider
import com.intellij.psi.PsiElement
import com.intellij.psi.PsiFile
import com.intellij.psi.TokenType
import com.intellij.psi.tree.IFileElementType
import com.intellij.psi.tree.TokenSet
import edu.umontreal.hatchery.psi.RosInterfaceFile
import edu.umontreal.hatchery.psi.RosInterfaceTypes

object RosInterfaceParserDefinition : ParserDefinition {
  val WHITE_SPACES = TokenSet.create(TokenType.WHITE_SPACE)
  val COMMENTS = TokenSet.create(RosInterfaceTypes.COMMENT)
  val FILE = IFileElementType(RosInterfaceLanguage)

  override fun createLexer(project: Project) = RosInterfaceLexerAdapter

  override fun getWhitespaceTokens(): TokenSet = WHITE_SPACES

  override fun getCommentTokens(): TokenSet = COMMENTS

  override fun getStringLiteralElements(): TokenSet = TokenSet.EMPTY

  override fun createParser(project: Project) = RosInterfaceParser()

  override fun getFileNodeType(): IFileElementType = FILE

  override fun createFile(viewProvider: FileViewProvider): PsiFile = RosInterfaceFile(viewProvider)

  override fun spaceExistenceTypeBetweenTokens(left: ASTNode, right: ASTNode) = SpaceRequirements.MAY

  override fun createElement(node: ASTNode): PsiElement = RosInterfaceTypes.Factory.createElement(node)
}