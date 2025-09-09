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
Unit uragdoll;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, utiming, upcontacts;

Const
  NUM_BONES = 12;
  NUM_JOINTS = 11;

Type

  { RagdollDemo }

  RagdollDemo = Class(RigidBodyApplication)
  private
  public
    Constructor Create(); override;

    Function GetTitle: String; override;

  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := RagdollDemo.Create;
End;

{ RagdollDemo }

Constructor RagdollDemo.Create;
Begin
  Inherited Create();
End;

Function RagdollDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Ragdoll Demo';
End;

End.

