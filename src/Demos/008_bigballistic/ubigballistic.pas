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
Unit ubigballistic;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp
  , urandom, ucollide_fine, ujoints, ucore, ubody, uprecision, utiming //, uparticle, , upcontacts
  ;

Type

  ShotType = (
    UNUSED = 0,
    PISTOL,
    ARTILLERY,
    FIREBALL,
    LASER
    );

  { AmmoRound }

  AmmoRound = Class(CollisionSphere)
  private
    _type: ShotType;
    startTime: QWord;
  public
    Constructor Create; override;
    Destructor Destroy; override;

    Procedure Render;
    Procedure setState(shotType: ShotType);

  End;

  { Box }

  Box = Class(CollisionBox)
  private
  public
    Constructor Create; override;
    Destructor Destroy; override;

    Procedure Render;
    Procedure setState(z: Float);
  End;

Const
  ammoRounds = 256;
  boxes = 2;

Type
  { BigBallisticDemo }

  BigBallisticDemo = Class(RigidBodyApplication)
  private
    ammo: Array[0..ammoRounds - 1] Of AmmoRound;
    boxData: Array[0..boxes - 1] Of Box;
    currentShotType: ShotType;
  protected

    (** Processes the contact generation code. *)
    Procedure generateContacts(); override;

    (** Processes the objects in the simulation forward in time. *)
    Procedure updateObjects(duration: real); override;
    //
    (** Resets the position of all the bones. *)
    Procedure reset(); override;

    (** Dispatches a round. *)
    Procedure fire();

  public
    Constructor Create(); override;
    Destructor Destroy(); override;

    Function GetWindowDimension: TPoint; override;

    (** Sets up the rendering. *)
    Procedure initGraphics(); override;

    (** Returns the window title for the demo. *)
    Function GetTitle: String; override;

    (** Display the particle positions. *)
    Procedure display(); override;

    (** Handle a mouse click. *)
    Procedure mouse(button, state, x, y: integer); override;

    (** Handles a key press. *)
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := BigBallisticDemo.Create;
End;

{ AmmoRound }

Constructor AmmoRound.Create;
Begin
  Inherited Create;
  body := RigidBody.Create();
End;

Destructor AmmoRound.Destroy;
Begin
  body.free;
  Inherited Destroy;
End;

Procedure AmmoRound.Render;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);

  glPushMatrix();
  glMultMatrixf(mat);
  glutSolidSphere(radius, 20, 20);
  glPopMatrix();
End;

Procedure AmmoRound.setState(shotType: ShotType);
Var
  tensor: Matrix3;
  coeff: Float;
Begin
  _type := shotType;

  // Set the properties of the particle
  Case _type Of

    PISTOL: Begin
        body.setMass(1.5);
        body.setVelocity(0.0, 0.0, 20.0);
        body.setAcceleration(0.0, -0.5, 0.0);
        body.setDamping(0.99, 0.8);
        radius := 0.2;
      End;

    ARTILLERY: Begin
        body.setMass(200.0); // 200.0kg
        body.setVelocity(0.0, 30.0, 40.0); // 50m/s
        body.setAcceleration(0.0, -21.0, 0.0);
        body.setDamping(0.99, 0.8);
        radius := 0.4;
      End;

    FIREBALL: Begin
        body.setMass(4.0); // 4.0kg - mostly blast damage
        body.setVelocity(0.0, -0.5, 10.0); // 10m/s
        body.setAcceleration(0.0, 0.3, 0.0); // Floats up
        body.setDamping(0.9, 0.8);
        radius := 0.6;

      End;

    LASER: Begin
        // Note that this is the kind of laser bolt seen in films,
        // not a realistic laser beam!
        body.setMass(0.1); // 0.1kg - almost no weight
        body.setVelocity(0.0, 0.0, 100.0); // 100m/s
        body.setAcceleration(0.0, 0.0, 0.0); // No gravity
        body.setDamping(0.99, 0.8);
        radius := 0.2;
      End;
  End;

  body.setCanSleep(false);
  body.setAwake();

  coeff := 0.4 * body.getMass() * radius * radius;
  tensor.setInertiaTensorCoeffs(coeff, coeff, coeff);
  body.setInertiaTensor(tensor);

  // Set the data common to all particle types
  body.setPosition(0.0, 1.5, 0.0);
  startTime := TimingData.lastFrameTimestamp;

  // Clear the force accumulators
  body.calculateDerivedData();
  calculateInternals();
End;

{ Box }

Constructor Box.Create;
Begin
  Inherited Create;
  body := RigidBody.Create();
End;

Destructor Box.Destroy;
Begin
  body.free;
  Inherited Destroy;
End;

Procedure Box.Render;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation

  body.getGLTransform(mat);

  glPushMatrix();
  glMultMatrixf(mat);
  glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Procedure Box.setState(z: Float);
Var
  mass: FLoat;
  tensor: Matrix3;
Begin
  body.setPosition(0, 3, z);
  body.setOrientation(1, 0, 0, 0);
  body.setVelocity(0, 0, 0);
  body.setRotation(V3(0, 0, 0));
  halfSize := V3(1, 1, 1);

  mass := halfSize.x * halfSize.y * halfSize.z * 8.0;
  body.setMass(mass);

  tensor.setBlockInertiaTensor(halfSize, mass);
  body.setInertiaTensor(tensor);

  body.setLinearDamping(0.95);
  body.setAngularDamping(0.8);
  body.clearAccumulators();
  body.setAcceleration(0, -10.0, 0);

  body.setCanSleep(false);
  body.setAwake();

  body.calculateDerivedData();
  calculateInternals();
End;

{ BigBallisticDemo }

Constructor BigBallisticDemo.Create;
Var
  i: Integer;
Begin
  Inherited Create();
  currentShotType := LASER;
  pauseSimulation := false;
  For i := 0 To high(ammo) Do Begin
    ammo[i] := AmmoRound.Create;
  End;
  For i := 0 To high(boxData) Do Begin
    boxdata[i] := Box.Create;
  End;
  reset();
End;

Destructor BigBallisticDemo.Destroy;
Var
  i: Integer;
Begin
  For i := 0 To high(ammo) Do Begin
    ammo[i].free;
  End;
  For i := 0 To high(boxData) Do Begin
    boxdata[i].free;
  End;
  Inherited Destroy();
End;

Function BigBallisticDemo.GetWindowDimension: TPoint;
Begin
  Result := point(640, 320);
End;

Procedure BigBallisticDemo.generateContacts;
Var
  plane: CollisionPlane;
  i, j: Integer;
Begin
  // Create the ground plane data
  plane := CollisionPlane.Create;
  plane.direction := V3(0, 1, 0);
  plane.offset := 0;

  // Set up the collision data structure
  cData.reset(maxContacts);
  cData.friction := 0.9;
  cData.restitution := 0.1;
  cData.tolerance := 0.1;

  // Check ground plane collisions
  //for (Box *box = boxData; box < boxData+boxes; box++)
  For i := 0 To high(boxData) Do Begin
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.boxAndHalfSpace(boxData[i], plane, cData);

    // Check for collisions with each shot
    For j := 0 To high(ammo) Do Begin
      If (ammo[j]._type <> UNUSED) Then Begin

        If (Not cData.hasMoreContacts()) Then exit;
        // When we get a collision, remove the shot
        If (CollisionDetector.boxAndSphere(boxData[i], ammo[j], cData) <> 0) Then Begin
          ammo[j]._type := UNUSED;
        End;
      End;
    End;
  End;

  // NB We aren't checking box-box collisions.
  plane.free;
End;

Procedure BigBallisticDemo.updateObjects(duration: real);
Var
  i: Integer;
Begin
  // Update the physics of each particle in turn

  For i := 0 To high(ammo) Do Begin
    If (ammo[i]._type <> UNUSED) Then Begin

      // Run the physics
      ammo[i].body.integrate(duration);
      ammo[i].calculateInternals();

      // Check if the particle is now invalid
      If (ammo[i].body.getPosition().y < 0.0) Or (
        ammo[i].startTime + 5000 < TimingData.lastFrameTimestamp) Or (
        ammo[i].body.getPosition().z > 200.0) Then Begin

        // We simply set the shot type to be unused, so the
        // memory it occupies can be reused by another shot.
        ammo[i]._type := UNUSED;
      End;
    End;
  End;

  // Update the boxes
  For i := 0 To high(boxData) Do Begin
    // Run the physics
    boxData[i].body.integrate(duration);
    boxData[i].calculateInternals();
  End;
End;

Procedure BigBallisticDemo.reset;
Var
  i: Integer;
  z: float;
Begin
  // Make all shots unused

  For i := 0 To high(ammo) Do Begin
    ammo[i]._type := UNUSED;
  End;

  // Initialise the box
  z := 20.0;
  //for (Box *box = boxData; box < boxData+boxes; box++)
  For i := 0 To high(boxData) Do Begin
    boxData[i].setState(z);
    z := z + 90.0;
  End;
End;

Procedure BigBallisticDemo.fire;
Var
  i: Integer;
Begin
  // Find the first available round.
  For i := 0 To high(ammo) Do Begin
    If ammo[i]._type = UNUSED Then Begin
      ammo[i].setState(currentShotType);
      exit;
    End;
  End;
End;

Procedure BigBallisticDemo.initGraphics;
Const
  lightAmbient: Array Of single = (0.8, 0.8, 0.8, 1.0);
  lightDiffuse: Array Of single = (0.9, 0.95, 1.0, 1.0);
Begin
  glLightfv(GL_LIGHT0, GL_AMBIENT, @lightAmbient[0]);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, @lightDiffuse[0]);

  glEnable(GL_LIGHT0);

  Inherited initGraphics();
End;

Function BigBallisticDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Big Ballistic Demo';
End;

Procedure BigBallisticDemo.display;
Const
  lightPosition: Array Of single = (-1, 1, 0, 0);
Var
  i: Integer;
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
  i := 0;
  While i < 200 Do Begin
    glVertex3f(-5.0, 0.0, i);
    glVertex3f(5.0, 0.0, i);
    inc(i, 10);
  End;
  glEnd();

  // Render each particle in turn
  glColor3f(1, 0, 0);
  For i := 0 To high(ammo) Do Begin
    If (ammo[i]._type <> UNUSED) Then Begin
      ammo[i].render();
    End;
  End;

  // Render the box
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_POSITION, @lightPosition[0]);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  glColor3f(1, 0, 0);
  For i := 0 To high(boxData) Do Begin
    boxData[i].render();
  End;
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);

  // Render the description
  glColor3f(0.0, 0.0, 0.0);
  renderText(10.0, 34.0, 'Click: Fire' + LineEnding + '1-4: Select Ammo');

  // Render the name of the current shot type
  Case (currentShotType) Of
    PISTOL: renderText(10.0, 10.0, 'Current Ammo: Pistol');
    ARTILLERY: renderText(10.0, 10.0, 'Current Ammo: Artillery');
    FIREBALL: renderText(10.0, 10.0, 'Current Ammo: Fireball');
    LASER: renderText(10.0, 10.0, 'Current Ammo: Laser');
  End;
End;

Procedure BigBallisticDemo.mouse(button, state, x, y: integer);
Begin
  // Fire the current weapon.
  If (state = GLUT_DOWN) Then fire();
End;

Procedure BigBallisticDemo.Key(akey: Char);
Begin
  Case (akey) Of
    '1': currentShotType := PISTOL;
    '2': currentShotType := ARTILLERY;
    '3': currentShotType := FIREBALL;
    '4': currentShotType := LASER;
    'r', 'R': reset();
  End;
End;

End.

