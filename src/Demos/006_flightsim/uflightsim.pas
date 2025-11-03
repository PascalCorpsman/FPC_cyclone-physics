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
Unit uflightsim;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, upcontacts, upfgen, utiming, ufgen, ubody;

Type

  { FlightSimDemo }

  FlightSimDemo = Class(Application)
  private
    left_wing: AeroControl;
    right_wing: AeroControl;
    rudder: AeroControl;
    tail: Aero;
    aircraft: RigidBody;
    registry: ForceRegistry;

    windspeed: Vector3;

    left_wing_control: float;
    right_wing_control: float;
    rudder_control: float;
    Procedure ResetPlane();
    Procedure drawAircraft();
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
  result := FlightSimDemo.Create;
End;

{ FlightSimDemo }

Constructor FlightSimDemo.Create;
Var
  it: Matrix3;
Begin
  Inherited create();
  aircraft := RigidBody.Create();
  right_wing := AeroControl.create(M3(0, 0, 0, -1, -0.5, 0, 0, 0, 0),
    M3(0, 0, 0, -0.995, -0.5, 0, 0, 0, 0),
    M3(0, 0, 0, -1.005, -0.5, 0, 0, 0, 0),
    V3(-1.0, 0.0, 2.0), @windspeed);

  left_wing := AeroControl.create(M3(0, 0, 0, -1, -0.5, 0, 0, 0, 0),
    M3(0, 0, 0, -0.995, -0.5, 0, 0, 0, 0),
    M3(0, 0, 0, -1.005, -0.5, 0, 0, 0, 0),
    V3(-1.0, 0.0, -2.0), @windspeed);

  rudder := AeroControl.create(M3(0, 0, 0, 0, 0, 0, 0, 0, 0),
    M3(0, 0, 0, 0, 0, 0, 0.01, 0, 0),
    M3(0, 0, 0, 0, 0, 0, -0.01, 0, 0),
    V3(2.0, 0.5, 0), @windspeed);

  tail := Aero.create(M3(0, 0, 0, -1, -0.5, 0, 0, 0, -0.1),
    V3(2.0, 0, 0), @windspeed);

  left_wing_control := 0;
  right_wing_control := 0;
  rudder_control := 0;

  windspeed.create(0, 0, 0);
  // Set up the aircraft rigid body.
  resetPlane();

  aircraft.setMass(2.5);

  it.setBlockInertiaTensor(V3(2, 1, 1), 1);
  aircraft.setInertiaTensor(it);

  aircraft.setDamping(0.8, 0.8);

  aircraft.setAcceleration(GRAVITY);
  aircraft.calculateDerivedData();

  aircraft.setAwake();
  aircraft.setCanSleep(false);

  registry := ForceRegistry.create;
  registry.add(@aircraft, @left_wing);
  registry.add(@aircraft, @right_wing);
  registry.add(@aircraft, @rudder);
  registry.add(@aircraft, @tail);
End;

Destructor FlightSimDemo.Destroy;
Begin
  aircraft.free;
  right_wing.free;
  left_wing.free;
  rudder.free;
  tail.free;
  registry.free;
  Inherited Destroy();
End;

Procedure FlightSimDemo.ResetPlane;
Begin
  aircraft.setPosition(0, 0, 0);
  aircraft.setOrientation(1, 0, 0, 0);

  aircraft.setVelocity(0, 0, 0);
  aircraft.setRotation(0, 0, 0);
End;

Procedure FlightSimDemo.drawAircraft();
Begin
  // Fuselage
  glPushMatrix();
  glTranslatef(-0.5, 0, 0);
  glScalef(2.0, 0.8, 1.0);
  glutSolidCube(1.0);
  glPopMatrix();

  // Rear Fuselage
  glPushMatrix();
  glTranslatef(1.0, 0.15, 0);
  glScalef(2.75, 0.5, 0.5);
  glutSolidCube(1.0);
  glPopMatrix();

  // Wings
  glPushMatrix();
  glTranslatef(0, 0.3, 0);
  glScalef(0.8, 0.1, 6.0);
  glutSolidCube(1.0);
  glPopMatrix();

  // Rudder
  glPushMatrix();
  glTranslatef(2.0, 0.775, 0);
  glScalef(0.75, 1.15, 0.1);
  glutSolidCube(1.0);
  glPopMatrix();

  // Tail-plane
  glPushMatrix();
  glTranslatef(1.9, 0, 0);
  glScalef(0.85, 0.1, 2.0);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Function FlightSimDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Flight Sim Demo';
End;

Procedure FlightSimDemo.Display;
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

  pos := aircraft.getPosition();
  offset.create(4.0 + aircraft.getVelocity().magnitude(), 0, 0);
  offset := aircraft.getTransform().transformDirection(offset);
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
  transform := aircraft.getTransform();
  transform.fillGLArray(gl_transform);
  glPushMatrix();
  glMultMatrixf(@gl_transform[0]);

  // Draw the aircraft
  glColor3f(0, 0, 0);
  drawAircraft();
  glPopMatrix();

  glColor3f(0.8, 0.8, 0.8);
  glPushMatrix();
  glTranslatef(0, -1.0 - pos.y, 0);
  glScalef(1.0, 0.001, 1.0);
  glMultMatrixf(gl_transform);
  drawAircraft();
  glPopMatrix();

  buffer :=
    format('Altitude: %.1f | Speed %.1f',
    [aircraft.getPosition().y,
    aircraft.getVelocity().magnitude()]
      );
  glColor3f(0, 0, 0);
  renderText(10.0, 24.0, buffer);

  buffer := format(
    'Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f',
    [left_wing_control, right_wing_control, rudder_control]
    );
  renderText(10.0, 10.0, buffer);
End;

Procedure FlightSimDemo.Update;
Var
  duration: Float;
  pos, propulsion: Vector3;
Begin
  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  // Start with no forces or acceleration.
  aircraft.clearAccumulators();

  // Add the propeller force
  propulsion.create(-10.0, 0, 0);
  propulsion := aircraft.getTransform().transformDirection(propulsion);
  aircraft.addForce(propulsion);

  // Add the forces acting on the aircraft.
  registry.updateForces(duration);

  // Update the aircraft's physics.
  aircraft.integrate(duration);

  // Do a very basic collision detection and response with the ground.
  pos := aircraft.getPosition();
  If (pos.y < 0.0) Then Begin

    pos.y := 0.0;
    aircraft.setPosition(pos);

    If (aircraft.getVelocity().y < -10.0) Then Begin
      resetPlane();
    End;
  End;

  Inherited update();
End;

Procedure FlightSimDemo.Key(akey: Char);
Begin
  Case (akey) Of
    'q', 'Q': Begin
        rudder_control := rudder_control + 0.1;
      End;
    'e', 'E': Begin
        rudder_control := rudder_control - 0.1;
      End;
    'w', 'W': Begin
        left_wing_control := left_wing_control - 0.1;
        right_wing_control := right_wing_control - 0.1;
      End;
    's', 'S': Begin
        left_wing_control := left_wing_control + 0.1;
        right_wing_control := right_wing_control + 0.1;
      End;
    'd', 'D': Begin
        left_wing_control := left_wing_control - 0.1;
        right_wing_control := right_wing_control + 0.1;
      End;
    'a', 'A': Begin
        left_wing_control := left_wing_control + 0.1;
        right_wing_control := right_wing_control - 0.1;
      End;

    'x', 'X': Begin
        left_wing_control := 0;
        right_wing_control := 0;
        rudder_control := 0;
      End;
    'r', 'R': resetPlane();
  Else
    Inherited key(akey);
  End;

  // Make sure the controls are in range
  If (left_wing_control < -1.0) Then
    left_wing_control := -1.0
  Else If (left_wing_control > 1.0) Then
    left_wing_control := 1.0;
  If (right_wing_control < -1.0) Then
    right_wing_control := -1.0
  Else If (right_wing_control > 1.0) Then
    right_wing_control := 1.0;
  If (rudder_control < -1.0) Then
    rudder_control := -1.0
  Else If (rudder_control > 1.0) Then
    rudder_control := 1.0;

  // Update the control surfaces
  left_wing.setControl(left_wing_control);
  right_wing.setControl(right_wing_control);
  rudder.setControl(rudder_control);
End;


End.

