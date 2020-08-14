package org.duckietown.hatchery.rosinterface;

import com.intellij.lexer.FlexLexer;
import com.intellij.psi.tree.IElementType;
import org.duckietown.hatchery.psi.RosInterfaceTypes;
import com.intellij.psi.TokenType;

%%

%class RosInterfaceLexer
%implements FlexLexer
%unicode
%function advance
%type IElementType

CRLF=\R
WHITE_SPACE=[\ \t]
VALUE_CHARACTER=[^\r\n\f#]
END_OF_LINE_COMMENT=#[^\r\n\f]*
SEPARATOR=[:=]
TYPE_CHARACTER=[^:=#\ \r\n\t\f\\]
KEY_CHARACTER=[^:=#\ \r\n\t\f\\]
TRIPLE_DASH=---

%state TYPE_STATE
%state KEY_STATE
%state WAITING_STATE
%state VALUE_STATE

%%

<YYINITIAL> {END_OF_LINE_COMMENT}               { yybegin(YYINITIAL); return RosInterfaceTypes.COMMENT; }

<YYINITIAL> {TYPE_CHARACTER}+                   { yybegin(TYPE_STATE); return RosInterfaceTypes.TYPE; }

<TYPE_STATE> {WHITE_SPACE}+                     { yybegin(KEY_STATE); return TokenType.WHITE_SPACE; }

<KEY_STATE> {KEY_CHARACTER}+                    { yybegin(WAITING_STATE); return RosInterfaceTypes.KEY; }

<WAITING_STATE> {WHITE_SPACE}+                  { yybegin(WAITING_STATE); return TokenType.WHITE_SPACE; }

<WAITING_STATE> {CRLF}({CRLF}|{WHITE_SPACE})+   { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

<WAITING_STATE> {END_OF_LINE_COMMENT}           { yybegin(YYINITIAL); return RosInterfaceTypes.COMMENT; }

<WAITING_STATE> {SEPARATOR}                     { yybegin(VALUE_STATE); return RosInterfaceTypes.SEPARATOR; }

<VALUE_STATE> {VALUE_CHARACTER}+                { yybegin(VALUE_STATE); return RosInterfaceTypes.VALUE; }

<VALUE_STATE> {END_OF_LINE_COMMENT}             { yybegin(VALUE_STATE); return RosInterfaceTypes.COMMENT; }

<VALUE_STATE> {CRLF}({CRLF}|{WHITE_SPACE})+     { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

({CRLF}|{WHITE_SPACE}|{TRIPLE_DASH})+           { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

[^]                                             { return TokenType.BAD_CHARACTER; }