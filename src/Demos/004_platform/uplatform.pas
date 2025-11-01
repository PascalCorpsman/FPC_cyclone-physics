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
Unit uplatform;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, uplinks;

Const
  ROD_COUNT = 15;

  BASE_MASS = 1;
  EXTRA_MASS = 10;

Type

  { PlatformDemo }

  PlatformDemo = Class(MassAggregateApplication)
  private
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
  result := PlatformDemo.Create;
End;

{ PlatformDemo }

Constructor PlatformDemo.Create();
Var
  i: Integer;
Begin
  Inherited create(6);
  rods := Nil;
  massPos.create(0, 0, 0.5);
  // Create the masses and connections.
  particleArray[0].setPosition(0, 0, 1);
  particleArray[1].setPosition(0, 0, -1);
  particleArray[2].setPosition(-3, 2, 1);
  particleArray[3].setPosition(-3, 2, -1);
  particleArray[4].setPosition(4, 2, 1);
  particleArray[5].setPosition(4, 2, -1);

  For i := 0 To 5 Do Begin
    particleArray[i].setMass(BASE_MASS);
    particleArray[i].setVelocity(0, 0, 0);
    particleArray[i].setDamping(0.9);
    particleArray[i].setAcceleration(GRAVITY);
    particleArray[i].clearAccumulator();
  End;
  setlength(rods, ROD_COUNT);
  For i := 0 To ROD_COUNT - 1 Do
    rods[i] := ParticleRod.Create;

  rods[0].particle[0] := @particleArray[0];
  rods[0].particle[1] := @particleArray[1];
  rods[0].length := 2;
  rods[1].particle[0] := @particleArray[2];
  rods[1].particle[1] := @particleArray[3];
  rods[1].length := 2;
  rods[2].particle[0] := @particleArray[4];
  rods[2].particle[1] := @particleArray[5];
  rods[2].length := 2;

  rods[3].particle[0] := @particleArray[2];
  rods[3].particle[1] := @particleArray[4];
  rods[3].length := 7;
  rods[4].particle[0] := @particleArray[3];
  rods[4].particle[1] := @particleArray[5];
  rods[4].length := 7;

  rods[5].particle[0] := @particleArray[0];
  rods[5].particle[1] := @particleArray[2];
  rods[5].length := 3.606;
  rods[6].particle[0] := @particleArray[1];
  rods[6].particle[1] := @particleArray[3];
  rods[6].length := 3.606;

  rods[7].particle[0] := @particleArray[0];
  rods[7].particle[1] := @particleArray[4];
  rods[7].length := 4.472;
  rods[8].particle[0] := @particleArray[1];
  rods[8].particle[1] := @particleArray[5];
  rods[8].length := 4.472;

  rods[9].particle[0] := @particleArray[0];
  rods[9].particle[1] := @particleArray[3];
  rods[9].length := 4.123;
  rods[10].particle[0] := @particleArray[2];
  rods[10].particle[1] := @particleArray[5];
  rods[10].length := 7.28;
  rods[11].particle[0] := @particleArray[4];
  rods[11].particle[1] := @particleArray[1];
  rods[11].length := 4.899;
  rods[12].particle[0] := @particleArray[1];
  rods[12].particle[1] := @particleArray[2];
  rods[12].length := 4.123;
  rods[13].particle[0] := @particleArray[3];
  rods[13].particle[1] := @particleArray[4];
  rods[13].length := 7.28;
  rods[14].particle[0] := @particleArray[5];
  rods[14].particle[1] := @particleArray[0];
  rods[14].length := 4.899;

  For i := 0 To ROD_COUNT - 1 Do Begin
    world.getContactGenerators().push_back(rods[i]);
  End;

  updateAdditionalMass();
End;

Destructor PlatformDemo.Destroy();
Var
  i: Integer;
Begin
  For i := 0 To high(rods) Do Begin
    rods[i].Free;
  End;
  Inherited Destroy();
End;

Procedure PlatformDemo.updateAdditionalMass();
Var
  i: unsigned;
  xp, zp: Float;
Begin
  For i := 2 To 5 Do Begin
    particleArray[i].setMass(BASE_MASS);
  End;

  // Find the coordinates of the mass as an index and proportion
  xp := massPos.x;
  If (xp < 0) Then xp := 0;
  If (xp > 1) Then xp := 1;

  zp := massPos.z;
  If (zp < 0) Then zp := 0;
  If (zp > 1) Then zp := 1;

  // Calculate where to draw the mass
  massDisplayPos.clear();

  // Add the proportion to the correct masses
  particleArray[2].setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * (1 - zp));
  massDisplayPos.addScaledVector(
    particleArray[2].getPosition(), (1 - xp) * (1 - zp)
    );

  If (xp > 0) Then Begin

    particleArray[4].setMass(BASE_MASS + EXTRA_MASS * xp * (1 - zp));
    massDisplayPos.addScaledVector(
      particleArray[4].getPosition(), xp * (1 - zp)
      );

    If (zp > 0) Then Begin
      particleArray[5].setMass(BASE_MASS + EXTRA_MASS * xp * zp);
      massDisplayPos.addScaledVector(
        particleArray[5].getPosition(), xp * zp
        );
    End;
  End;
  If (zp > 0) Then Begin
    particleArray[3].setMass(BASE_MASS + EXTRA_MASS * (1 - xp) * zp);
    massDisplayPos.addScaledVector(
      particleArray[3].getPosition(), (1 - xp) * zp
      );
  End;
End;

Function PlatformDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Platform Demo';
End;

Procedure PlatformDemo.Display;
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
  glEnd();
  glColor3f(1, 0, 0);
  glPushMatrix();
  glTranslatef(massDisplayPos.x, massDisplayPos.y + 0.25, massDisplayPos.z);
  glutSolidSphere(0.25, 20, 10);
  glPopMatrix();
End;

Procedure PlatformDemo.Update;
Begin
  Inherited update();
  updateAdditionalMass();
End;

Procedure PlatformDemo.Key(akey: Char);
Begin
  Case akey Of
    's', 'S': Begin
        massPos.z := massPos.z - 0.1;
        If (massPos.z < 0.0) Then massPos.z := 0.0;
      End;
    'w', 'W': Begin
        massPos.z := massPos.z + 0.1;
        If (massPos.z > 1.0) Then massPos.z := 1.0;
      End;
    'a', 'A': Begin
        massPos.x := massPos.x - 0.1;
        If (massPos.x < 0.0) Then massPos.x := 0.0;
      End;
    'd', 'D': Begin
        massPos.x := massPos.x + 0.1;
        If (massPos.x > 1.0) Then massPos.x := 1.0;
      End;
  Else
    Inherited key(akey);
  End;
End;

End.

