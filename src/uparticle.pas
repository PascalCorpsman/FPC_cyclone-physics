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
Unit uparticle;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision;

Type

  { Particle }

  Particle = Object
  protected
    inverseMass: float; // = 1 / mass -> 0 = not movable
    damping: float; // Quasi die diletantische emulation eines "luftwiderstandes" 1 = deaktiviert, 0 = alles kaputt -> 0.999 = OK ;)

    position: Vector3;
    velocity: Vector3;
    forceAccum: Vector3;
    acceleration: Vector3;
  public
    Procedure Integrate(duration: float);

    Procedure setMass(aMass: float);
    Function getMass(): float;

    Procedure setInverseMass(aInverseMass: float);
    Function getInverseMass(): float;

    Function hasFiniteMass: Boolean;

    Procedure setDamping(aDamping: float);
    Function getDamping(): float;

    Procedure setPosition(Const aPosition: Vector3); overload;
    Procedure setPosition(x, y, z: float); overload;
    Procedure getPosition(Out aPosition: Vector3); overload;
    Function getPosition(): Vector3; overload;

    Procedure setVelocity(Const aVelocity: Vector3); overload;
    Procedure setVelocity(x, y, z: float); overload;
    Procedure getVelocity(Out aVelocity: Vector3); overload;
    Function getVelocity(): Vector3; overload;

    Procedure setAcceleration(Const aAccelereration: Vector3); overload;
    Procedure setAcceleration(x, y, z: float); overload;
    Procedure getAcceleration(Out aAccelereration: Vector3); overload;
    Function getAcceleration(): Vector3; overload;

    Procedure clearAccumulator();

    Procedure addForce(Const aForce: Vector3);
  End;

  PParticle = ^Particle;

Implementation

{ Particle }

Procedure Particle.clearAccumulator;
Begin
  forceAccum.Clear;
End;

Procedure Particle.addForce(Const aForce: Vector3);
Begin
  acceleration := acceleration + aForce;
End;

Procedure Particle.setInverseMass(aInverseMass: float);
Begin
  inverseMass := aInverseMass;
End;

Function Particle.getInverseMass(): float;
Begin
  result := inverseMass;
End;

Function Particle.hasFiniteMass: Boolean;
Begin
  result := inverseMass >= 0.0;
End;

Procedure Particle.setDamping(aDamping: float);
Begin
  damping := aDamping;
End;

Function Particle.getDamping(): float;
Begin
  result := damping;
End;

Procedure Particle.setPosition(Const aPosition: Vector3);
Begin
  position := aPosition;
End;

Procedure Particle.setPosition(x, y, z: float);
Begin
  position.create(x, y, z);
End;

Procedure Particle.getPosition(Out aPosition: Vector3);
Begin
  aPosition := position;
End;

Function Particle.getPosition(): Vector3;
Begin
  result := position;
End;

Procedure Particle.setVelocity(Const aVelocity: Vector3);
Begin
  velocity := aVelocity;
End;

Procedure Particle.setVelocity(x, y, z: float);
Begin
  velocity.create(x, y, z);
End;

Procedure Particle.getVelocity(Out aVelocity: Vector3);
Begin
  aVelocity := velocity;
End;

Function Particle.getVelocity(): Vector3;
Begin
  result := velocity;
End;

Procedure Particle.setAcceleration(Const aAccelereration: Vector3);
Begin
  acceleration := aAccelereration;
End;

Procedure Particle.setAcceleration(x, y, z: float);
Begin
  acceleration.create(x, y, z);
End;

Procedure Particle.getAcceleration(Out aAccelereration: Vector3);
Begin
  aAccelereration := acceleration;
End;

Function Particle.getAcceleration(): Vector3;
Begin
  result := acceleration;
End;

Procedure Particle.setMass(aMass: float);
Begin
  assert(amass <> 0);
  inverseMass := 1 / aMass;
End;

Function Particle.getMass(): float;
Begin
  If (inverseMass = 0) Then Begin
    result := REAL_MAX;
  End
  Else Begin
    result := 1. / inverseMass;
  End;
End;

Procedure Particle.Integrate(duration: float);
Var
  resultingAcc: Vector3;
Begin
  If inverseMass <= 0 Then exit;
  assert(duration > 0);
  position.addScaledVector(velocity, duration);
  resultingAcc := acceleration;
  resultingAcc.addScaledVector(forceAccum, inverseMass);

  velocity.addScaledVector(resultingAcc, duration);

  velocity := velocity * real_pow(damping, duration);
  clearAccumulator();
End;

End.

