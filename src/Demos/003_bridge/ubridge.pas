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
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, utiming, upcontacts;

Const
  ROD_COUNT = 6;
  CABLE_COUNT = 10;
  SUPPORT_COUNT = 12;
  BASE_MASS = 1;
  EXTRA_MASS = 10;

Type

  { BridgeDemo }

  BridgeDemo = Class(MassAggregateApplication)
  private
    //cyclone::ParticleCableConstraint *supports;
    //cyclone::ParticleCable *cables;
    //cyclone::ParticleRod *rods;

    massPos: Vector3;
    massDisplayPos: Vector3;

    (**
     * Updates particle masses to take into account the mass
     * that's crossing the bridge.
     *)
    //void updateAdditionalMass();

  public
    Constructor Create(); virtual; reintroduce;
    Destructor Destroy(); override;

    Function GetTitle: String; override;

    Procedure Display; override;
    Procedure Update; override;
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := BridgeDemo.Create;
End;

{ BridgeDemo }

Constructor BridgeDemo.Create;
Begin
  Inherited Create(12);
  //cables(0),
  //supports(0),
  //rods(0),
  massPos.create(0, 0, 0.5);
End;

Destructor BridgeDemo.Destroy;
Begin
  Inherited Destroy();
End;

Function BridgeDemo.GetTitle: String;
Begin
  result := 'Cyclone > Bridge Demo';
End;

Procedure BridgeDemo.Display;
Begin

End;

Procedure BridgeDemo.Update;
Begin

End;

Procedure BridgeDemo.Key(akey: Char);
Begin

End;


End.

