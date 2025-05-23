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
Unit core;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, precision;

Type

  { Vector3 }

  Vector3 = Object
  private
    pad: float; // Not used at all, just to pad the size to 16 byte instead 12 when using float = single
  public
    x, y, z: float;

    Constructor create(); overload;
    Constructor create(aX, aY, aZ: float); overload;

    Procedure Invert;
  End;

Implementation

{ Vector3 }

Constructor Vector3.create;
Begin
  x := 0;
  y := 0;
  z := 0;
End;

Constructor Vector3.create(aX, aY, aZ: Single);
Begin
  x := aX;
  y := aY;
  z := aZ;

End;

Procedure Vector3.Invert;
Begin
  x := -x;
  y := -y;
  z := -z;
End;

End.

