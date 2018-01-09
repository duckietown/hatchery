// This is a generated file. Not intended for manual editing.
package edu.umontreal.hatchery.parser;

import com.intellij.lang.PsiBuilder;
import com.intellij.lang.PsiBuilder.Marker;
import static edu.umontreal.hatchery.psi.SimpleTypes.*;
import static com.intellij.lang.parser.GeneratedParserUtilBase.*;
import com.intellij.psi.tree.IElementType;
import com.intellij.lang.ASTNode;
import com.intellij.psi.tree.TokenSet;
import com.intellij.lang.PsiParser;
import com.intellij.lang.LightPsiParser;

@SuppressWarnings({"SimplifiableIfStatement", "UnusedAssignment"})
public class SimpleParser implements PsiParser, LightPsiParser {

  public ASTNode parse(IElementType t, PsiBuilder b) {
    parseLight(t, b);
    return b.getTreeBuilt();
  }

  public void parseLight(IElementType t, PsiBuilder b) {
    boolean r;
    b = adapt_builder_(t, b, this, null);
    Marker m = enter_section_(b, 0, _COLLAPSE_, null);
    if (t == PROPERTY) {
      r = property(b, 0);
    }
    else {
      r = parse_root_(t, b, 0);
    }
    exit_section_(b, 0, m, t, r, true, TRUE_CONDITION);
  }

  protected boolean parse_root_(IElementType t, PsiBuilder b, int l) {
    return rosmsgFile(b, l + 1);
  }

  /* ********************************************************** */
  // (TYPE SEPARATOR KEY SEPARATOR VALUE?)|(TYPE KEY)
  public static boolean property(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "property")) return false;
    boolean r;
    Marker m = enter_section_(b, l, _NONE_, PROPERTY, "<property>");
    r = property_0(b, l + 1);
    if (!r) r = property_1(b, l + 1);
    exit_section_(b, l, m, r, false, recover_property_parser_);
    return r;
  }

  // TYPE SEPARATOR KEY SEPARATOR VALUE?
  private static boolean property_0(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "property_0")) return false;
    boolean r;
    Marker m = enter_section_(b);
    r = consumeTokens(b, 0, TYPE, SEPARATOR, KEY, SEPARATOR);
    r = r && property_0_4(b, l + 1);
    exit_section_(b, m, null, r);
    return r;
  }

  // VALUE?
  private static boolean property_0_4(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "property_0_4")) return false;
    consumeToken(b, VALUE);
    return true;
  }

  // TYPE KEY
  private static boolean property_1(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "property_1")) return false;
    boolean r;
    Marker m = enter_section_(b);
    r = consumeTokens(b, 0, TYPE, KEY);
    exit_section_(b, m, null, r);
    return r;
  }

  /* ********************************************************** */
  // !(KEY|SEPARATOR|COMMENT)
  static boolean recover_property(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "recover_property")) return false;
    boolean r;
    Marker m = enter_section_(b, l, _NOT_);
    r = !recover_property_0(b, l + 1);
    exit_section_(b, l, m, r, false, null);
    return r;
  }

  // KEY|SEPARATOR|COMMENT
  private static boolean recover_property_0(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "recover_property_0")) return false;
    boolean r;
    Marker m = enter_section_(b);
    r = consumeToken(b, KEY);
    if (!r) r = consumeToken(b, SEPARATOR);
    if (!r) r = consumeToken(b, COMMENT);
    exit_section_(b, m, null, r);
    return r;
  }

  /* ********************************************************** */
  // (property|COMMENT)*
  static boolean rosmsgFile(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "rosmsgFile")) return false;
    int c = current_position_(b);
    while (true) {
      if (!rosmsgFile_0(b, l + 1)) break;
      if (!empty_element_parsed_guard_(b, "rosmsgFile", c)) break;
      c = current_position_(b);
    }
    return true;
  }

  // property|COMMENT
  private static boolean rosmsgFile_0(PsiBuilder b, int l) {
    if (!recursion_guard_(b, l, "rosmsgFile_0")) return false;
    boolean r;
    Marker m = enter_section_(b);
    r = property(b, l + 1);
    if (!r) r = consumeToken(b, COMMENT);
    exit_section_(b, m, null, r);
    return r;
  }

  final static Parser recover_property_parser_ = new Parser() {
    public boolean parse(PsiBuilder b, int l) {
      return recover_property(b, l + 1);
    }
  };
}
