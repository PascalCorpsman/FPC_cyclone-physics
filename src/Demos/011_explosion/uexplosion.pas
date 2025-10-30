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
Unit uexplosion;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp
  , urandom, ucollide_fine, ucore, ubody, uprecision //, uparticle, utiming, upcontacts
  ;


Const
  OBJECTS = 5;
  floorMirror: TOpenGLMatrix =
  (
    1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
    );
Type

  { Ball }

  Ball = Class(CollisionSphere)
  private
  public
    Constructor Create(); override;
    Destructor Destroy; override;
    Procedure Render;
    Procedure renderShadow;
    Procedure SetState(Const position: Vector3;
      Const orientation: Quaternion;
      Const aradius: float;
      Const velocity: Vector3);
    Procedure random(Const random: TRandom);
  End;

  { box }

  box = Class(CollisionBox)
  private

  public
    isOverlapping: Boolean;
    Constructor Create(); override;
    Destructor Destroy; override;
    Procedure Render;
    Procedure renderShadow;
    Procedure SetState(Const position: Vector3;
      Const orientation: Quaternion;
      Const extents: Vector3;
      Const velocity: Vector3);
    Procedure random(Const random: TRandom);
  End;

  { ExplosionDemo }

  ExplosionDemo = Class(RigidBodyApplication)
  private
    editMode, upMode: Boolean;
    boxes: unsigned;
    boxData: Array Of Box;
    balls: unsigned;
    ballData: Array Of Ball;
    Procedure fire;
  protected

    (** Processes the contact generation code. *)
    Procedure generateContacts(); override;

    (** Processes the objects in the simulation forward in time. *)
    Procedure updateObjects(duration: real); override;
    //
    (** Resets the position of all the bones. *)
    Procedure reset(); override;

  public
    Constructor Create(); override;
    Destructor Destroy(); override;

    (** Sets up the rendering. *)
    Procedure initGraphics(); override;

    (** Returns the window title for the demo. *)
    Function GetTitle: String; override;

    (** Display the particle positions. *)
    Procedure display(); override;

    (** Handle a mouse drag *)
    Procedure mouseDrag(x, y: integer); override;

    (** Handles a key press. *)
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := ExplosionDemo.Create;
End;

{ Ball }

Constructor Ball.Create;
Begin
  Inherited Create();
  body := RigidBody.Create();
End;

Destructor Ball.Destroy;
Begin
  body.free;
  Inherited Destroy;
End;

(** Draws the box, excluding its shadow. *)

Procedure Ball.Render;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);

  If (body.getAwake()) Then
    glColor3f(1.0, 0.7, 0.7)
  Else
    glColor3f(0.7, 0.7, 1.0);

  glPushMatrix();
  glMultMatrixf(mat);
  glutSolidSphere(radius, 20, 20);
  glPopMatrix();
End;

Procedure Ball.renderShadow;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);
  glPushMatrix();
  glScalef(1.0, 0, 1.0);
  glMultMatrixf(mat);
  glutSolidSphere(radius, 20, 20);
  glPopMatrix();
End;

Procedure Ball.SetState(Const position: Vector3; Const orientation: Quaternion;
  Const aradius: float; Const velocity: Vector3);
Var
  coeff, mass: float;
  tensor: Matrix3;
Begin
  body.setPosition(position);
  body.setOrientation(orientation);
  body.setVelocity(velocity);
  body.setRotation(V3(0, 0, 0));
  radius := aradius;

  mass := 4.0 * 0.3333 * 3.1415 * radius * radius * radius;
  body.setMass(mass);

  coeff := 0.4 * mass * radius * radius;
  tensor.setInertiaTensorCoeffs(coeff, coeff, coeff);
  body.setInertiaTensor(tensor);

  body.setLinearDamping(0.95);
  body.setAngularDamping(0.8);
  body.clearAccumulators();
  body.setAcceleration(0, -10.0, 0);

  //body->setCanSleep(false);
  body.setAwake();

  body.calculateDerivedData();
End;

Procedure Ball.random(Const random: TRandom);
Var
  minPos: Vector3;
  maxPos: Vector3;
Begin
  minPos := v3(-5, 5, -5);
  maxPos := v3(5, 10, 5);
  setState(
    random.randomVector(minPos, maxPos),
    random.randomQuaternion(),
    random.randomReal(0.5, 1.5),
    V3(0, 0, 0)
    );
End;

{ box }

Constructor box.Create;
Begin
  Inherited Create();
  body := RigidBody.Create();
  isOverlapping := false;
End;

Destructor box.Destroy;
Begin
  body.free;
  Inherited Destroy;
End;

Procedure box.Render;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);

  If (isOverlapping) Then
    glColor3f(0.7, 1.0, 0.7)
  Else If (body.getAwake()) Then
    glColor3f(1.0, 0.7, 0.7)
  Else
    glColor3f(0.7, 0.7, 1.0);

  glPushMatrix();
  glMultMatrixf(mat);
  glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Procedure box.renderShadow;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);
  glPushMatrix();
  glScalef(1.0, 0, 1.0);
  glMultMatrixf(mat);
  glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Procedure box.SetState(Const position: Vector3; Const orientation: Quaternion;
  Const extents: Vector3; Const velocity: Vector3);
Var
  mass: float;
  tensor: Matrix3;
Begin
  body.setPosition(position);
  body.setOrientation(orientation);
  body.setVelocity(velocity);
  body.setRotation(V3(0, 0, 0));
  halfSize := extents;

  mass := halfSize.x * halfSize.y * halfSize.z * 8.0;
  body.setMass(mass);

  tensor.setBlockInertiaTensor(halfSize, mass);
  body.setInertiaTensor(tensor);

  body.setLinearDamping(0.95);
  body.setAngularDamping(0.8);
  body.clearAccumulators();
  body.setAcceleration(0, -10.0, 0);

  body.setAwake();

  body.calculateDerivedData();
End;

Procedure box.random(Const random: TRandom);
Var
  minPos, maxPos, minSize, maxSize: Vector3;
Begin
  minPos := v3(-5, 5, -5);
  maxPos := v3(5, 10, 5);
  minSize := v3(0.5, 0.5, 0.5);
  maxSize := v3(4.5, 1.5, 1.5);

  setState(
    random.randomVector(minPos, maxPos),
    random.randomQuaternion(),
    random.randomVector(minSize, maxSize),
    V3(0, 0, 0)
    );
End;

{ ExplosionDemo }

Procedure ExplosionDemo.fire;
Var
  pos: Vector3;
Begin
  pos := ballData[0].body.getPosition();
  pos.Normalize();

  ballData[0].body.addForce(pos * -1000.0); // Force to center..
End;

Procedure ExplosionDemo.generateContacts;
Var
  plane: CollisionPlane;
  i, other: Integer;
Begin
  // Note that this method makes a lot of use of early returns to avoid
  // processing lots of potential contacts that it hasn't got room to
  // store.

  // Create the ground plane data
  plane := CollisionPlane.Create;
  plane.direction := V3(0, 1, 0);
  plane.offset := 0;

  // Set up the collision data structure
  cData.reset(maxContacts);
  cData.friction := 0.9;
  cData.restitution := 0.6;
  cData.tolerance := 0.1;

  // Perform exhaustive collision detection

  For i := 0 To high(boxData) Do Begin

    // Check for collisions with the ground plane
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.boxAndHalfSpace(boxData[i], plane, cData);

    // Check for collisions with each other box
    For other := i + 1 To high(boxData) Do Begin
      If (Not cData.hasMoreContacts()) Then exit;
      CollisionDetector.boxAndBox(boxData[i], boxData[other], cData);

      If (IntersectionTests.boxAndBox(boxData[i], boxData[other])) Then Begin
        boxData[i].isOverlapping := true;
        boxData[other].isOverlapping := true;
      End;
    End;

    // Check for collisions with each ball
    For other := 0 To high(ballData) Do Begin
      If (Not cData.hasMoreContacts()) Then exit;
      CollisionDetector.boxAndSphere(boxData[i], ballData[other], cData);
    End;
  End;

  For i := 0 To high(balldata) Do Begin
    // Check for collisions with the ground plane
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.sphereAndHalfSpace(balldata[i], plane, cData);


    For other := i + 1 To high(ballData) Do Begin
      // Check for collisions with the ground plane
      If (Not cData.hasMoreContacts()) Then exit;
      CollisionDetector.sphereAndSphere(balldata[i], balldata[other], cData);
    End;
  End;
  plane.free;
End;

Procedure ExplosionDemo.updateObjects(duration: real);
Var
  i: Integer;
Begin
  // Update the physics of each box in turn
  For i := 0 To high(boxData) Do Begin
    // Run the physics
    boxData[i].body.integrate(duration);
    boxData[i].calculateInternals();
    boxData[i].isOverlapping := false;
  End;

  // Update the physics of each ball in turn
  For i := 0 To high(ballData) Do Begin
    // Run the physics
    ballData[i].body.integrate(duration);
    ballData[i].calculateInternals();
  End;
End;

Procedure ExplosionDemo.reset;
Var
  q: Quaternion;
  i: Integer;
Begin
  q.Create;

  boxData[0].setState(V3(0, 3, 0),
    q,
    V3(4, 1, 1),
    V3(0, 1, 0));

  If (boxes > 1) Then Begin
    q.Create(1.0, 0.1, 0.05, 0.01);
    boxData[1].setState(V3(0, 4.75, 2),
      q,
      V3(1, 1, 4),
      V3(0, 1, 0));
  End;

  // Create the random objects
  For i := 2 To boxes - 1 Do Begin
    boxData[i].random(random);
  End;

  For i := 0 To balls - 1 Do Begin
    ballData[i].random(random);
  End;

  // Reset the contacts
  cData.contactCount := 0;

End;

Constructor ExplosionDemo.Create;
Var
  i: integer;
Begin
  Inherited Create();
  boxes := OBJECTS;
  setlength(boxData, boxes);
  balls := OBJECTS;
  setlength(ballData, balls);
  editMode := false;
  upMode := false;
  For i := 0 To high(boxData) Do Begin
    boxData[i] := box.Create();
  End;
  For i := 0 To high(ballData) Do Begin
    ballData[i] := Ball.Create();
  End;
  // Set up the initial positions
  reset();
End;

Destructor ExplosionDemo.Destroy;
Var
  i: Integer;
Begin
  For i := 0 To high(boxData) Do Begin
    boxData[i].free;
  End;
  For i := 0 To high(ballData) Do Begin
    ballData[i].free;
  End;
  Inherited Destroy();
End;

Procedure ExplosionDemo.initGraphics;
Const
  lightAmbient: Array Of single = (0.8, 0.8, 0.8, 1.0);
  lightDiffuse: Array Of single = (0.9, 0.95, 1.0, 1.0);

Begin
  glLightfv(GL_LIGHT0, GL_AMBIENT, @lightAmbient[0]);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, @lightDiffuse[0]);

  glEnable(GL_LIGHT0);
  Inherited initGraphics();
End;

Function ExplosionDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Explosion Demo';
End;

Procedure ExplosionDemo.display;
Const
  lightPosition: Array Of single = (1, -1, 0, 0);
  lightPositionMirror: Array Of single = (1, 1, 0, 0);
Var
  i, j: Integer;
  _theta: Single;
Begin

  // Update the transform matrices of each box in turn
  For i := 0 To high(boxData) Do Begin
    boxData[i].calculateInternals();
    boxData[i].isOverlapping := false;
  End;

  // Update the transform matrices of each ball in turn
  For i := 0 To high(ballData) Do Begin
    // Run the physics
    ballData[i].calculateInternals();
  End;

  // Clear the viewport and set the camera direction
  Inherited display();

  // Render each element in turn as a shadow
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_POSITION, @lightPosition[0]);
  glPushMatrix();
  glMultMatrixf(floorMirror);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  For i := 0 To high(boxData) Do Begin
    boxData[i].Render;
  End;
  For i := 0 To high(ballData) Do Begin
    ballData[i].Render;
  End;
  glPopMatrix();
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);

  // Draw some scale circles
  glColor3f(0.75, 0.75, 0.75);
  For i := 1 To 19 Do Begin
    glBegin(GL_LINE_LOOP);
    For j := 0 To 31 Do Begin
      _theta := 3.1415926 * j / 16.0;
      glVertex3f(i * real_cos(_theta), 0.0, i * real_sin(_theta));
    End;
    glEnd();
  End;
  glBegin(GL_LINES);
  glVertex3f(-20, 0, 0);
  glVertex3f(20, 0, 0);
  glVertex3f(0, 0, -20);
  glVertex3f(0, 0, 20);
  glEnd();

  // Render each shadow in turn
  glEnable(GL_BLEND);
  glColor4f(0, 0, 0, 0.1);
  glDisable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  For i := 0 To high(boxData) Do Begin
    boxData[i].renderShadow;
  End;
  For i := 0 To high(ballData) Do Begin
    ballData[i].renderShadow;
  End;
  glDisable(GL_BLEND);

  // Render the boxes themselves
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_POSITION, @lightPositionMirror[0]);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  For i := 0 To high(boxData) Do Begin
    boxData[i].Render;
  End;
  For i := 0 To high(ballData) Do Begin
    ballData[i].Render;
  End;
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // Finish the frame, rendering any additional information
  drawDebug();
End;

Procedure ExplosionDemo.mouseDrag(x, y: integer);
Begin
  If (editMode) Then Begin
    boxData[0].body.setPosition(boxData[0].body.getPosition() +
      V3(
      (x - last_x) * 0.125,
      0,
      (y - last_y) * 0.125
      )
      );
    boxData[0].body.calculateDerivedData();
  End
  Else If (upMode) Then Begin
    boxData[0].body.setPosition(boxData[0].body.getPosition() +
      V3(
      0,
      (y - last_y) * 0.125,
      0
      )
      );
    boxData[0].body.calculateDerivedData();
  End
  Else Begin
    Inherited mouseDrag(x, y);
  End;

  // Remember the position
  last_x := x;
  last_y := y;

End;

Procedure ExplosionDemo.Key(akey: Char);
Var
  i: Integer;
Begin
  Case (akey) Of
    'e', 'E': Begin
        editMode := Not editMode;
        upMode := false;
        exit;
      End;

    't', 'T': Begin
        upMode := Not upMode;
        editMode := false;
        exit;
      End;

    'f', 'F': Begin // Not part of the orig example, ..
        Fire;
      End;

    'w', 'W': Begin
        For i := 0 To high(boxData) Do
          boxData[i].body.setAwake();
        For i := 0 To high(ballData) Do
          ballData[i].body.setAwake();
        exit;
      End;
  End;
  Inherited key(akey);
End;

End.

