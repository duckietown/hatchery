package edu.umontreal.hatchery;

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

CRLF=\R
WHITE_SPACE=[\ \n\t\f]
VALUE_CHARACTER=[^ \n\f\\] | "\\"{CRLF} | "\\".
END_OF_LINE_COMMENT=("#"|"!")[^\r\n]*
SEPARATOR=[:=]
KEY_CHARACTER=[^:=\ \n\t\f\\] | "\\ "

%state WAITING_VALUE

%%

<YYINITIAL> {END_OF_LINE_COMMENT}                      { yybegin(YYINITIAL); return ROSInterfaceTypes.COMMENT; }

<YYINITIAL> {KEY_CHARACTER}+                           {yybegin(YYINITIAL); return ROSInterfaceTypes.KEY; }

<YYINITIAL> {SEPARATOR}                                {yybegin(WAITING_VALUE); return ROSInterfaceTypes.SEPARATOR; }

<WAITING_VALUE> {CRLF}({CRLF}|{WHITE_SPACE})+          {yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

<WAITING_VALUE> {WHITE_SPACE}+                         { yybegin(WAITING_VALUE); return TokenType.WHITE_SPACE; }

<WAITING_VALUE> {VALUE_CHARACTER}*                     { yybegin(YYINITIAL); return ROSInterfaceTypes.VALUE; }

({CRLF}|{WHITE_SPACE})+                                { yybegin(YYINITIAL); return TokenType.WHITE_SPACE; }

.                                                      { return TokenType.BAD_CHARACTER; }