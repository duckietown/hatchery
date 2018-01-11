package edu.umontreal.hatchery.rosinterface;

import com.intellij.lexer.FlexLexer;
import com.intellij.psi.tree.IElementType;
import edu.umontreal.hatchery.psi.ROSInterfaceTypes;
import com.intellij.psi.TokenType;

%%

%class ROSInterfaceLexer
%implements FlexLexer
%unicode
%function advance
%type IElementType
%eof{  return;
%eof}

CRLF=[\R\n\f]
WHITE_SPACE=[\ \t]
VALUE_CHARACTER=[^ \n\f\\] | "\\"{CRLF} | "\\".
END_OF_LINE_COMMENT=("#"|"!")[^\r\n]*
SEPARATOR=[:=]
TYPE_CHARACTER=[^:=\ \n\t\f\\] | "\\ "
KEY_CHARACTER=[^:=\ \n\t\f\\] | "\\ "

%state WAITING_STATE
%state KEY_STATE
%state VALUE_STATE

%%

<YYINITIAL> {END_OF_LINE_COMMENT}               { yybegin(YYINITIAL); return ROSInterfaceTypes.COMMENT; }

<YYINITIAL> {TYPE_CHARACTER}+                   { yybegin(YYINITIAL); return ROSInterfaceTypes.TYPE; }

<YYINITIAL> {WHITE_SPACE}+                      { yybegin(KEY_STATE); return TokenType.WHITE_SPACE; }

<KEY_STATE> {KEY_CHARACTER}+                    { yybegin(KEY_STATE); return ROSInterfaceTypes.KEY; }

<KEY_STATE> {WHITE_SPACE}+                      { yybegin(WAITING_STATE); return TokenType.WHITE_SPACE; }

<KEY_STATE> {SEPARATOR}                         { yybegin(VALUE_STATE); return ROSInterfaceTypes.SEPARATOR; }

<WAITING_STATE> {CRLF}({CRLF}|{WHITE_SPACE})+   { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

<WAITING_STATE> {SEPARATOR}*                    { yybegin(VALUE_STATE); return ROSInterfaceTypes.SEPARATOR; }

<VALUE_STATE> {WHITE_SPACE}+                    { yybegin(VALUE_STATE); return TokenType.WHITE_SPACE; }

<VALUE_STATE> {VALUE_CHARACTER}*                { yybegin(VALUE_STATE); return ROSInterfaceTypes.VALUE; }

<VALUE_STATE> {CRLF}({CRLF}|{WHITE_SPACE})+     { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

({CRLF}|{WHITE_SPACE})+                         { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

.                                               { return TokenType.BAD_CHARACTER; }