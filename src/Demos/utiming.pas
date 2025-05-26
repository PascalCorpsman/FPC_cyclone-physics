(******************************************************************************)
(*                                                                            *)
(* Author      : Uwe Schächterle (Corpsman)                                   *)
(*                                                                            *)
(* This file is part of FPC_cyclone-physics-demos                             *)
(*                                                                            *)
(*  See the file license.md, located under:                                   *)
(*  https://github.com/PascalCorpsman/Software_Licenses/blob/main/license.md  *)
(*  for details about the license.                                            *)
(*                                                                            *)
(*               It is not allowed to change or remove this text from any     *)
(*               source file of the project.                                  *)
(*                                                                            *)
(******************************************************************************)
Unit utiming;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils;

Type

  { TTimingData }

  TTimingData = Class
  private

  public

    lastFrameDuration: QWord;
    lastFrameTimestamp: QWord;

    Constructor Create;
    Procedure Update;
  End;

Var
  TimingData: TTimingData = Nil;

Implementation

{ TTimingData }

Constructor TTimingData.Create;
Begin
  Update;
End;

Procedure TTimingData.Update;
Var
  n: QWord;
Begin
  n := GetTickCount64;
  lastFrameDuration := n - lastFrameTimestamp;
  lastFrameTimestamp := n;
End;

End.

