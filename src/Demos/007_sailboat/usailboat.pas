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
Unit usailboat;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, urandom, utiming, ufgen, ubody;

Type

  { SailboatDemo }

  SailboatDemo = Class(Application)
  private
    buoyancy: ufgen.Buoyancy;

    sail: Aero;
    sailboat: RigidBody;
    registry: ForceRegistry;
    windspeed: Vector3;

    sail_control: float;
    Procedure drawBoat;
  public
    Constructor Create(); virtual; reintroduce;
    Destructor Destroy(); override;

    Function GetWindowDimension: TPoint; override;


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
  result := SailboatDemo.Create;
End;

{ SailboatDemo }

Constructor SailboatDemo.Create;
Var
  it: Matrix3;
Begin
  Inherited create();
  sailboat := RigidBody.Create();

  sail := Aero.create(M3(0, 0, 0, 0, 0, 0, 0, 0, -1.0), V3(2.0, 0, 0), @windspeed);

  buoyancy := ufgen.Buoyancy.Create(V3(0.0, 0.5, 0.0), 1.0, 3.0, 1.6);

  sail_control := 0;

  windspeed.create(0, 0, 0);

  // Set up the boat's rigid body.
  sailboat.setPosition(0, 1.6, 0);
  sailboat.setOrientation(1, 0, 0, 0);

  sailboat.setVelocity(0, 0, 0);
  sailboat.setRotation(0, 0, 0);

  sailboat.setMass(200.0);

  it.setBlockInertiaTensor(V3(2, 1, 1), 100.0);
  sailboat.setInertiaTensor(it);

  sailboat.setDamping(0.8, 0.8);

  sailboat.setAcceleration(GRAVITY);
  sailboat.calculateDerivedData();

  sailboat.setAwake();
  sailboat.setCanSleep(false);

  registry := ForceRegistry.create;
  registry.add(@sailboat, @sail);
  registry.add(@sailboat, @buoyancy);
End;

Destructor SailboatDemo.Destroy;
Begin
  sailboat.free;
  sail.free;
  buoyancy.free;
  registry.free;
  Inherited Destroy();
End;

Function SailboatDemo.GetWindowDimension: TPoint;
Begin
  Result := point(640, 320);
End;

Procedure SailboatDemo.drawBoat;
Begin
  // Left Hull
  glPushMatrix();
  glTranslatef(0, 0, -1.0);
  glScalef(2.0, 0.4, 0.4);
  glutSolidCube(1.0);
  glPopMatrix();

  // Right Hull
  glPushMatrix();
  glTranslatef(0, 0, 1.0);
  glScalef(2.0, 0.4, 0.4);
  glutSolidCube(1.0);
  glPopMatrix();

  // Deck
  glPushMatrix();
  glTranslatef(0, 0.3, 0);
  glScalef(1.0, 0.1, 2.0);
  glutSolidCube(1.0);
  glPopMatrix();

  // Mast
  glPushMatrix();
  glTranslatef(0, 1.8, 0);
  glScalef(0.1, 3.0, 0.1);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Function SailboatDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Sail Boat Demo';
End;

Procedure SailboatDemo.Display;
Var
  pos, offset: Vector3;
  bx, bz: integer;
  x, z: integer;
  transform: Matrix4;
  gl_transform: TOpenGLMatrix;
  buffer: String;
Begin
  // Clear the view port and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  pos := sailboat.getPosition();
  offset.create(4.0, 0, 0);
  offset := sailboat.getTransform().transformDirection(offset);
  gluLookAt(pos.x + offset.x, pos.y + 5.0, pos.z + offset.z,
    pos.x, pos.y, pos.z,
    0.0, 1.0, 0.0);

  glColor3f(0.6, 0.6, 0.6);
  bx := trunc(pos.x);
  bz := trunc(pos.z);
  glBegin(GL_QUADS);
  For x := -20 To 19 Do Begin
    For z := -20 To 19 Do Begin
      glVertex3f(bx + x - 0.1, 0, bz + z - 0.1);
      glVertex3f(bx + x - 0.1, 0, bz + z + 0.1);
      glVertex3f(bx + x + 0.1, 0, bz + z + 0.1);
      glVertex3f(bx + x + 0.1, 0, bz + z - 0.1);
    End;
  End;
  glEnd();

  // Set the transform matrix for the aircraft
  transform := sailboat.getTransform();

  transform.fillGLArray(gl_transform);
  glPushMatrix();
  glMultMatrixf(@gl_transform[0]);

  // Draw the boat
  glColor3f(0, 0, 0);
  drawBoat();
  glPopMatrix();

  buffer := format(
    'Speed %.1f',
    [sailboat.getVelocity().magnitude()]
    );
  glColor3f(0, 0, 0);
  renderText(10.0, 24.0, buffer);

  buffer := format(
    'Sail Control: %.1f',
    [sail_control]
    );
  renderText(10.0, 10.0, buffer);
End;

Procedure SailboatDemo.Update;
Var
  duration: Float;
Begin
  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  // Start with no forces or acceleration.
  sailboat.clearAccumulators();

  // Add the forces acting on the boat.
  registry.updateForces(duration);

  // Update the boat's physics.
  sailboat.integrate(duration);

  // Change the wind speed.
  windspeed := windspeed * 0.9 + random.randomXZVector(1.0);

  Inherited update();
End;

Procedure SailboatDemo.Key(akey: Char);
Begin
  Case (akey) Of
    'q', 'Q': sail_control := sail_control - 0.1;
    'e', 'E': sail_control := sail_control + 0.1;
    'w', 'W': sail_control := 0;

  Else
    Inherited key(akey);
  End;

  // Make sure the controls are in range
  If (sail_control < -1.0) Then
    sail_control := -1.0
  Else If (sail_control > 1.0) Then
    sail_control := 1.0;

  // Update the control surfaces
  // sail.setControl(sail_control); -- This can only be set if sail was of type AeroControl
End;

End.

