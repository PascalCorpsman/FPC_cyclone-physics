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
Unit ubridge;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, utiming;

Const
  ruleCount = 9;
  maxFireworks = 1024;

Type

  { BridgeDemo }

  BridgeDemo = Class(MassAggregateApplication)


    Function GetTitle: String;

  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := BridgeDemo.Create(12);
End;

{ BridgeDemo }

Function BridgeDemo.GetTitle: String;
Begin
  result := 'Cyclone > Bridge Demo';
End;


End.

