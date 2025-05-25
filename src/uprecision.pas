(******************************************************************************)
(*                                                                            *)
(* Author      : Uwe Schächterle (Corpsman)                                   *)
(*                                                                            *)
(* This file is part of FPC_cyclone-physics                                   *)
(*                                                                            *)
(*  See the file license.md, located under:                                   *)
(*  https://github.com/PascalCorpsman/Software_Licenses/blob/main/license.md  *)
(*  for details about the license.                                            *)
(*                                                                            *)
(*               It is not allowed to change or remove this text from any     *)
(*               source file of the project.                                  *)
(*                                                                            *)
(******************************************************************************)
Unit uprecision;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, math;

(*
 * Default is single precision to be 1:1 compatible with OpenGL
 *)
{.$ DOUBLE_PRECISION}

{$IFDEF DOUBLE_PRECISION}

Const
  REAL_MAX = MaxDouble;
  REAL_Epsilon = 1E-12; // Taken from math.pas ( DZeroResolution )

Type
  float = Double; // use single in order to be OpenGL compatible !

{$ELSE}

Const
  REAL_MAX = MaxSingle;
  REAL_Epsilon = 1E-4; // Taken from math.pas ( SZeroResolution )

Type
  float = single; // use single in order to be OpenGL compatible !

Function real_pow(base, exp: float): Float;

{$ENDIF}

Implementation

Function real_pow(base, exp: float): Float;
Begin
  result := Power(base, exp);
End;

End.

