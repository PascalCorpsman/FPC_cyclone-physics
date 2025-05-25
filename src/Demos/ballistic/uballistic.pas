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
Unit uballistic;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, uparticle;

Type

  ShotType = (unused, pistol, artillery, FIREBALL, laser);

  { AmmoRound }

  AmmoRound = Object

    particle: uparticle.Particle;
    _type: ShotType;
    startTime: QWord;
    Procedure Render;
  End;

Const
  ammoRounds = 16;

Type

  { BallisticDemo }

  BallisticDemo = Class(Application)
  private
    ammo: Array[0..ammoRounds] Of AmmoRound;
    currentShotType: ShotType;
    Procedure fire();
  public
    Constructor Create(); override;

    Function GetTitle: String; override;
    Procedure Update; override;
    Procedure Display; override;
    Procedure mouse(button, state, x, y: integer); override;
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL, ucore, uOpenGL_ASCII_Font, utiming;

Function getApplication(): Application;
Begin
  result := BallisticDemo.Create;
End;

{ AmmoRound }

Procedure AmmoRound.Render;
Var
  position: Vector3;
Begin
  particle.getPosition(position);
  glColor3f(0, 0, 0);
  glPushMatrix();
  glTranslatef(position.x, position.y, position.z);
  glutSolidSphere(0.3, 5, 4);
  glPopMatrix();
  glColor3f(0.75, 0.75, 0.75);
  glPushMatrix();
  glTranslatef(position.x, 0, position.z);
  glScalef(1.0, 0.1, 1.0);
  glutSolidSphere(0.6, 5, 4);
  glPopMatrix();
End;

{ BallisticDemo }

Procedure BallisticDemo.fire;
Var
  i, shot: integer;
Begin
  // Find the first available round.
  shot := -1;
  For i := 0 To high(ammo) Do Begin
    If ammo[i]._type = unused Then Begin
      shot := i;
      break;
    End;
  End;
  // If we didn't find a round, then exit - we can't fire.
  If shot = -1 Then exit;

  // Set the properties of the particle
  Case (currentShotType) Of

    PISTOL: Begin
        ammo[shot].particle.setMass(2.0); // 2.0kg
        ammo[shot].particle.setVelocity(0.0, 0.0, 35.0); // 35m/s
        ammo[shot].particle.setAcceleration(0.0, -1.0, 0.0);
        ammo[shot].particle.setDamping(0.99);
      End;

    ARTILLERY: Begin
        ammo[shot].particle.setMass(200.0); // 200.0kg
        ammo[shot].particle.setVelocity(0.0, 30.0, 40.0); // 50m/s
        ammo[shot].particle.setAcceleration(0.0, -20.0, 0.0);
        ammo[shot].particle.setDamping(0.99);
      End;

    FIREBALL: Begin
        ammo[shot].particle.setMass(1.0); // 1.0kg - mostly blast damage
        ammo[shot].particle.setVelocity(0.0, 0.0, 10.0); // 5m/s
        ammo[shot].particle.setAcceleration(0.0, 0.6, 0.0); // Floats up
        ammo[shot].particle.setDamping(0.9);
      End;

    LASER: Begin
        // Note that this is the kind of laser bolt seen in films,
        // not a realistic laser beam!
        ammo[shot].particle.setMass(0.1); // 0.1kg - almost no weight
        ammo[shot].particle.setVelocity(0.0, 0.0, 100.0); // 100m/s
        ammo[shot].particle.setAcceleration(0.0, 0.0, 0.0); // No gravity
        ammo[shot].particle.setDamping(0.99);
      End;
  End;
  // Set the data common to all particle types
  ammo[shot].particle.setPosition(0.0, 1.5, 0.0);
  ammo[shot].startTime := TimingData.lastFrameTimestamp;
  ammo[shot]._type := currentShotType;

  // Clear the force accumulators
  ammo[shot].particle.clearAccumulator();
End;

Constructor BallisticDemo.Create;
Var
  i: Integer;
Begin
  Inherited Create();
  currentShotType := laser;
  For i := 0 To high(ammo) Do Begin
    ammo[i]._type := unused;
  End;
End;

Function BallisticDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Ballistic Demo';
End;

Procedure BallisticDemo.Update;
Var
  duration: Single;
  shot: Integer;
Begin
  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  // Update the physics of each particle in turn
  For shot := 0 To high(ammo) Do Begin
    If (ammo[shot]._type <> UNUSED) Then Begin
      // Run the physics
      ammo[shot].particle.integrate(duration);
      // Check if the particle is now invalid
      If (ammo[shot].particle.getPosition().y < 0.0) Or
        (ammo[shot].startTime + 5000 < TimingData.lastFrameTimestamp) Or
        (ammo[shot].particle.getPosition().z > 200.0) Then Begin
        // We simply set the shot type to be unused, so the
        // memory it occupies can be reused by another shot.
        ammo[shot]._type := UNUSED;
      End;
    End;
  End;
  Inherited Update;
End;

Procedure BallisticDemo.Display;
Var
  i, shot: Integer;
Begin
  // Clear the viewport and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(-25.0, 8.0, 5.0, 0.0, 5.0, 22.0, 0.0, 1.0, 0.0);

  // Draw a sphere at the firing point, and add a shadow projected
  // onto the ground plane.
  glColor3f(0.0, 0.0, 0.0);
  glPushMatrix();
  glTranslatef(0.0, 1.5, 0.0);
  glutSolidSphere(0.1, 5, 5);
  glTranslatef(0.0, -1.5, 0.0);
  glColor3f(0.75, 0.75, 0.75);
  glScalef(1.0, 0.1, 1.0);
  glutSolidSphere(0.1, 5, 5);
  glPopMatrix();

  // Draw some scale lines
  glColor3f(0.75, 0.75, 0.75);
  glBegin(GL_LINES);
  //      for (unsigned i = 0; i < 200; i += 10)
  i := 0;
  While i < 200 Do Begin
    glVertex3f(-5.0, 0.0, i);
    glVertex3f(5.0, 0.0, i);
    i := i + 10;
  End;
  glEnd();

  // Render each particle in turn
  For shot := 0 To high(ammo) Do Begin
    If (ammo[shot]._type <> UNUSED) Then Begin
      ammo[shot].render();
    End;
  End;

  // Render the description
  glColor3f(0, 0, 0);
  renderText(10.0, 34.0, 'Click: Fire' + LineEnding + '1 - 4: Select Ammo');

  // Render the name of the current shot type
  Case (currentShotType) Of
    PISTOL: renderText(10.0, 10.0, 'Current Ammo: Pistol');
    ARTILLERY: renderText(10.0, 10.0, 'Current Ammo: Artillery');
    FIREBALL: renderText(10.0, 10.0, 'Current Ammo: Fireball');
    LASER: renderText(10.0, 10.0, 'Current Ammo: Laser');
  End;
End;

Procedure BallisticDemo.mouse(button, state, x, y: integer);
Begin
  If state = GLUT_DOWN Then fire;
End;

Procedure BallisticDemo.Key(akey: Char);
Begin
  Case (akey) Of
    '1': currentShotType := PISTOL;
    '2': currentShotType := ARTILLERY;
    '3': currentShotType := FIREBALL;
    '4': currentShotType := LASER;
  End;
End;

End.

