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
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, utiming, upcontacts, uplinks;

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
    supports: Array Of ParticleCableConstraint;
    cables: Array Of ParticleCable;
    rods: Array Of ParticleRod;

    massPos: Vector3;
    massDisplayPos: Vector3;

    (**
     * Updates particle masses to take into account the mass
     * that's crossing the bridge.
     *)
    Procedure updateAdditionalMass();

  public
    Constructor Create(); virtual; reintroduce;
    Destructor Destroy(); override;

    (** Returns the window title for the demo. *)
    Function GetTitle: String; override;

    (** Display the particles. *)
    Procedure Display; override;

    (** Update the particle positions. *)
    Procedure Update; override;

    (** Handle a key press. *)
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

Procedure BridgeDemo.updateAdditionalMass();
Var
  x, i, z: integer;
  zp, xp: float;
Begin
  For i := 0 To high(particleArray) Do Begin
    particleArray[i].setMass(BASE_MASS);
  End;

  // Find the coordinates of the mass as an index and proportion
  x := trunc(massPos.x);
  xp := real_mod(massPos.x, 1.0);
  If (x < 0) Then Begin
    x := 0;
    xp := 0;
  End;
  If (x >= 5) Then Begin
    x := 5;
    xp := 0;
  End;

  z := trunc(massPos.z);
  zp := real_mod(massPos.z, 1.0);
  If (z < 0) Then Begin
    z := 0;
    zp := 0;
  End;
  If (z >= 1) Then Begin
    z := 1;
    zp := 0;
  End;

  // Calculate where to draw the mass
  massDisplayPos.clear();

  // Add the proportion to the correct masses
  particleArray[x * 2 + z].setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * (1 - zp));
  massDisplayPos.addScaledVector(
    particleArray[x * 2 + z].getPosition(), (1 - xp) * (1 - zp)
    );

  If (xp > 0) Then Begin
    particleArray[x * 2 + z + 2].setMass(BASE_MASS + EXTRA_MASS * xp * (1 - zp));
    massDisplayPos.addScaledVector(
      particleArray[x * 2 + z + 2].getPosition(), xp * (1 - zp)
      );

    If (zp > 0) Then Begin
      particleArray[x * 2 + z + 3].setMass(BASE_MASS + EXTRA_MASS * xp * zp);
      massDisplayPos.addScaledVector(
        particleArray[x * 2 + z + 3].getPosition(), xp * zp
        );
    End;
  End;
  If (zp > 0) Then Begin

    particleArray[x * 2 + z + 1].setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * zp);
    massDisplayPos.addScaledVector(
      particleArray[x * 2 + z + 1].getPosition(), (1 - xp) * zp
      );
  End;
End;

Constructor BridgeDemo.Create;
Var
  i: Integer;
Begin
  Inherited Create(12);
  cables := Nil;
  supports := Nil;
  rods := Nil;
  massPos.create(0, 0, 0.5);

  // Create the masses and connections.
  For i := 0 To 11 Do Begin
    particleArray[i].setPosition(
      (i Div 2) * 2.0 - 5.0,
      4,
      (i Mod 2) * 2.0 - 1.0
      );
    particleArray[i].setVelocity(0, 0, 0);
    particleArray[i].setDamping(0.9);
    particleArray[i].setAcceleration(GRAVITY);
    particleArray[i].clearAccumulator();
  End;

  // Add the links
  setlength(cables, CABLE_COUNT);
  For i := 0 To CABLE_COUNT - 1 Do Begin
    cables[i] := ParticleCable.create;
    cables[i].particle[0] := @particleArray[i];
    cables[i].particle[1] := @particleArray[i + 2];
    cables[i].maxLength := 1.9;
    cables[i].restitution := 0.3;
    world.getContactGenerators().push_back(cables[i]);
  End;

  setlength(supports, SUPPORT_COUNT);

  For i := 0 To SUPPORT_COUNT - 1 Do Begin
    supports[i] := ParticleCableConstraint.create;
    supports[i].particle := @particleArray[i];
    supports[i].anchor := V3(
      (i Div 2) * 2.2 - 5.5,
      6,
      (i Mod 2) * 1.6 - 0.8
      );
    If (i < 6) Then
      supports[i].maxLength := (i Div 2) * 0.5 + 3.0
    Else
      supports[i].maxLength := 5.5 - (i Div 2) * 0.5;
    supports[i].restitution := 0.5;
    world.getContactGenerators().push_back(supports[i]);
  End;

  setlength(rods, ROD_COUNT);
  For i := 0 To ROD_COUNT - 1 Do Begin
    rods[i] := ParticleRod.create;
    rods[i].particle[0] := @particleArray[i * 2];
    rods[i].particle[1] := @particleArray[i * 2 + 1];
    rods[i].length := 2;
    world.getContactGenerators().push_back(rods[i]);
  End;

  updateAdditionalMass();
End;

Destructor BridgeDemo.Destroy;
Var
  i: Integer;
Begin
  For i := 0 To high(cables) Do Begin
    cables[i].free;
  End;
  setlength(cables, 0);
  For i := 0 To high(supports) Do Begin
    supports[i].free;
  End;
  setlength(supports, 0);
  For i := 0 To high(rods) Do Begin
    rods[i].free;
  End;
  setlength(rods, 0);
  Inherited Destroy();
End;

Function BridgeDemo.GetTitle: String;
Begin
  result := 'Cyclone > Bridge Demo';
End;

Procedure BridgeDemo.Display;
Var
  particles: Array Of pparticle;
  i: Integer;
  p0, p1: Vector3;
Begin
  Inherited display();

  glBegin(GL_LINES);
  glColor3f(0, 0, 1);

  For i := 0 To ROD_COUNT - 1 Do Begin
    particles := rods[i].particle;
    p0 := particles[0]^.getPosition();
    p1 := particles[1]^.getPosition();
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
  End;

  glColor3f(0, 1, 0);
  For i := 0 To CABLE_COUNT - 1 Do Begin
    particles := cables[i].particle;
    p0 := particles[0]^.getPosition();
    p1 := particles[1]^.getPosition();
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
  End;

  glColor3f(0.7, 0.7, 0.7);
  For i := 0 To SUPPORT_COUNT - 1 Do Begin
    p0 := supports[i].particle^.getPosition();
    p1 := supports[i].anchor;
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
  End;
  glEnd();

  glColor3f(1, 0, 0);
  glPushMatrix();
  glTranslatef(massDisplayPos.x, massDisplayPos.y + 0.25, massDisplayPos.z);
  glutSolidSphere(0.25, 20, 10);
  glPopMatrix();
End;

Procedure BridgeDemo.Update;
Begin
  Inherited update();
  updateAdditionalMass();
End;

Procedure BridgeDemo.Key(akey: Char);
Begin
  Case akey Of
    's', 'S': Begin
        massPos.z := massPos.z + 0.1;
        If (massPos.z > 1.0) Then massPos.z := 1.0;
      End;
    'w', 'W': Begin
        massPos.z := massPos.z - 0.1;
        If (massPos.z < 0.0) Then massPos.z := 0.0;
      End;
    'a', 'A': Begin
        massPos.x := massPos.x - 0.1;
        If (massPos.x < 0.0) Then massPos.x := 0.0;
      End;
    'd', 'D': Begin
        massPos.x := massPos.x + 0.1;
        If (massPos.x > 5.0) Then massPos.x := 5.0;
      End;
  Else
    Inherited key(akey);
  End;
End;


End.

