// This is a generated file. Not intended for manual editing.
package edu.umontreal.hatchery.psi;

import com.intellij.psi.tree.IElementType;
import com.intellij.psi.PsiElement;
import com.intellij.lang.ASTNode;
import edu.umontreal.hatchery.psi.impl.*;

public interface SimpleTypes {

  IElementType PROPERTY = new SimpleElementType("PROPERTY");

  IElementType COMMENT = new SimpleTokenType("COMMENT");
  IElementType KEY = new SimpleTokenType("KEY");
  IElementType SEPARATOR = new SimpleTokenType("SEPARATOR");
  IElementType TYPE = new SimpleTokenType("TYPE");
  IElementType VALUE = new SimpleTokenType("VALUE");

  class Factory {
    public static PsiElement createElement(ASTNode node) {
      IElementType type = node.getElementType();
       if (type == PROPERTY) {
        return new SimplePropertyImpl(node);
      }
      throw new AssertionError("Unknown element type: " + type);
    }
  }
}
