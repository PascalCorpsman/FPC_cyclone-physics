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
Unit ujoints;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucontacts, ubody, ucore, uprecision;

Type

  { Joint }

  Joint = Class(ContactGenerator)
  public
    (**
     * Holds the two rigid bodies that are connected by this joint.
     *)
    body: Array[0..1] Of PRigidBody;

    (**
     * Holds the relative location of the connection for each
     * body, given in local coordinates.
     *)
    position: Array[0..1] Of PVector3;

    (**
     * Holds the maximum displacement at the joint before the
     * joint is considered to be violated. This is normally a
     * small, epsilon value.  It can be larger, however, in which
     * case the joint will behave as if an inelastic cable joined
     * the bodies at their joint locations.
     *)
    error: float;

    (**
     * Configures the joint in one go.
     *)
    Procedure _set(
      a: PRigidBody; a_pos: PVector3;
      b: PRigidBody; b_pos: PVector3;
      _Error: float
      );

    (**
     * Generates the contacts required to restore the joint if it
     * has been violated.
     *)
    Function addContact(Const contact: PContact; limit: unsigned): unsigned; override;
  End;

Implementation

{ Joint }

Procedure Joint._set(a: PRigidBody; a_pos: PVector3; b: PRigidBody;
  b_pos: PVector3; _Error: float);
Begin
  body[0] := a;
  body[1] := b;

  position[0] := a_pos;
  position[1] := b_pos;

  error := _error;
End;

Function Joint.addContact(Const contact: PContact; limit: unsigned): unsigned;
Var
  a_pos_world, b_pos_world,
    a_to_b, normal: Vector3;
  length: float;
Begin
  result := 0;
  // Calculate the position of each connection point in world coordinates
  a_pos_world := body[0]^.getPointInWorldSpace(position[0]);
  b_pos_world := body[1]^.getPointInWorldSpace(position[1]);

  // Calculate the length of the joint
  a_to_b := b_pos_world - a_pos_world;
  normal := a_to_b;
  normal.Normalize();
  length := a_to_b.magnitude();

  // Check if it is violated
  If (real_abs(length) > error) Then Begin
    contact^.body[0] := body[0];
    contact^.body[1] := body[1];
    contact^.contactNormal := normal;
    contact^.contactPoint := (a_pos_world + b_pos_world) * 0.5;
    contact^.penetration := length - error;
    contact^.friction := 1.0;
    contact^.restitution := 0;
    result := 1;
  End;
End;

End.

