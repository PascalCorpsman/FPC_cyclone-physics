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
Unit ufracture;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp
  , urandom, ucollide_fine, ucore, ubody, uprecision, ucontacts
  ;


Const
  MAX_BLOCKS = 9;

Type

  PBlock = ^Block;

  { Block }

  Block = Class(CollisionBox)
  private
    exists: boolean;
  public
    Constructor Create; override;
    Destructor Destroy; override;

    Procedure Render;

    Procedure setState(Const position: Vector3;
      Const orientation: Quaternion;
      Const extents: Vector3;
      Const velocity: Vector3);

    (**
     * Calculates and sets the mass and inertia tensor of this block,
     * assuming it has the given constant density.
     *)
    Procedure calculateMassProperties(invDensity: float);

    (**
     * Performs the division of the given block into four, writing the
     * eight new blocks into the given blocks array. The blocks array can be
     * a pointer to the same location as the target pointer: since the
     * original block is always deleted, this effectively reuses its storage.
     * The algorithm is structured to allow this reuse.
     *)
    Procedure divideBlock(Const contact: pContact;
      target: PBlock; blocks: PBlock);

  End;


  { FractureDemo }

  FractureDemo = Class(RigidBodyApplication)
  private
    hit: Boolean;
    ball_active: Boolean;
    fracture_contact: unsigned;

    (** Holds the bodies. *)
    blocks: Array[0..MAX_BLOCKS - 1] Of Block;

    (** Holds the projectile. *)
    ball: CollisionSphere;
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

    (** Update the objects. *)
    Procedure update(); override;

    (** Returns the window title for the demo. *)
    Function GetTitle: String; override;

    (** Display the particle positions. *)
    Procedure display(); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := FractureDemo.Create;
End;

{ Block }

Constructor Block.Create;
Begin
  Inherited Create;
  exists := false;
  body := RigidBody.Create();
End;

Destructor Block.Destroy;
Begin
  body.free;
  Inherited Destroy;
End;

Procedure Block.Render;
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
  glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Procedure Block.setState(Const position: Vector3;
  Const orientation: Quaternion; Const extents: Vector3; Const velocity: Vector3
  );
Var
  mass: Float;
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

  //body->setCanSleep(false);
  body.setAwake();

  body.calculateDerivedData();
End;

Procedure Block.calculateMassProperties(invDensity: float);
Var
  tensor, m: Matrix3;
  volume, mass: Float;
Begin
  // Check for infinite mass
  If (invDensity <= 0) Then Begin

    // Just set zeros for both mass and inertia tensor
    body.setInverseMass(0);
    m.Create();
    body.setInverseInertiaTensor(m);
  End
  Else Begin
    // Otherwise we need to calculate the mass
    volume := halfSize.magnitude() * 2.0;
    mass := volume / invDensity;

    body.setMass(mass);

    // And calculate the inertia tensor from the mass and size
    mass := mass * 0.333;

    tensor.setInertiaTensorCoeffs(
      mass * halfSize.y * halfSize.y + halfSize.z * halfSize.z,
      mass * halfSize.y * halfSize.x + halfSize.z * halfSize.z,
      mass * halfSize.y * halfSize.x + halfSize.z * halfSize.y
      );
    body.setInertiaTensor(tensor);
  End;
End;

Procedure Block.divideBlock(Const contact: pContact; target: PBlock;
  blocks: PBlock);
Var
  normal: Vector3;
  _body: PRigidBody;
  size, point: Vector3;
  tempBody: RigidBody;
  invDensity: float;
  i: Integer;
  direction, _halfSize, newPos,
    _min, _max: Vector3;
  m4: Matrix4;
Begin
  // Find out if we're block one or two in the contact structure, and
  // therefore what the contact normal is.
  normal := contact^.contactNormal;
  _body := contact^.body[0];
  If (_body <> @target^.body) Then Begin
    normal.invert();
    _body := contact^.body[1];
  End;

  // Work out where on the body (in body coordinates) the contact is
  // and its direction.
  point := _body^.getPointInLocalSpace(contact^.contactPoint);
  normal := _body^.getDirectionInLocalSpace(normal);

  // Work out the centre of the split: this is the point coordinates
  // for each of the axes perpendicular to the normal, and 0 for the
  // axis along the normal.
  point := point - normal * (point * normal);

  // Take a copy of the half size, so we can create the new blocks.
  size := target^.halfSize;

  // Take a copy also of the body's other data.
  tempBody := RigidBody.create;
  tempBody.setPosition(_body^.getPosition());
  tempBody.setOrientation(_body^.getOrientation());
  tempBody.setVelocity(_body^.getVelocity());
  tempBody.setRotation(_body^.getRotation());
  tempBody.setLinearDamping(_body^.getLinearDamping());
  tempBody.setAngularDamping(_body^.getAngularDamping());
  tempBody.setInverseInertiaTensor(_body^.getInverseInertiaTensor());
  tempBody.calculateDerivedData();

  // Remove the old block
  target^.exists := false;

  // Work out the inverse density of the old block
  invDensity := halfSize.magnitude() * 8 * _body^.getInverseMass();

  // Now split the block into eight.
  For i := 0 To 7 Do Begin

    // Find the minimum and maximum extents of the new block
    // in old-block coordinates

    If ((i And 1) = 0) Then Begin
      _min.x := -size.x;
      _max.x := point.x;
    End
    Else Begin
      _min.x := point.x;
      _max.x := size.x;
    End;
    If ((i And 2) = 0) Then Begin
      _min.y := -size.y;
      _max.y := point.y;
    End
    Else Begin
      _min.y := point.y;
      _max.y := size.y;
    End;
    If ((i And 4) = 0) Then Begin
      _min.z := -size.z;
      _max.z := point.z;
    End
    Else Begin
      _min.z := point.z;
      _max.z := size.z;
    End;

    // Get the origin and half size of the block, in old-body
    // local coordinates.
    _halfSize := (_max - _min) * 0.5;
    newPos := _halfSize + _min;

    // Convert the origin to world coordinates.
    newPos := tempBody.getPointInWorldSpace(newPos);

    // Work out the direction to the impact.
    direction := newPos - contact^.contactPoint;
    direction.Normalize();

    // Set the body's properties (we assume the block has a body
    // already that we're going to overwrite).
    blocks[i].body.setPosition(newPos);
    blocks[i].body.setVelocity(tempBody.getVelocity() + direction * 10.0);
    blocks[i].body.setOrientation(tempBody.getOrientation());
    blocks[i].body.setRotation(tempBody.getRotation());
    blocks[i].body.setLinearDamping(tempBody.getLinearDamping());
    blocks[i].body.setAngularDamping(tempBody.getAngularDamping());
    blocks[i].body.setAwake(true);
    blocks[i].body.setAcceleration(GRAVITY);
    blocks[i].body.clearAccumulators();
    blocks[i].body.calculateDerivedData();
    m4.create();
    blocks[i].offset := m4;
    blocks[i].exists := true;
    blocks[i].halfSize := _halfSize;

    // Finally calculate the mass and inertia tensor of the new block
    blocks[i].calculateMassProperties(invDensity);
  End;
  tempBody.free;
End;

{ FractureDemo }

Constructor FractureDemo.Create;
Var
  it: Matrix3;
  i: Integer;
Begin
  Inherited Create();
  // Create the ball.
  ball := CollisionSphere.Create();
  ball.body := RigidBody.Create();
  ball.radius := 0.25;
  ball.body.setMass(5.0);
  ball.body.setDamping(0.9, 0.9);

  it.setDiagonal(5.0, 5.0, 5.0);
  ball.body.setInertiaTensor(it);
  ball.body.setAcceleration(GRAVITY);

  ball.body.setCanSleep(false);
  ball.body.setAwake(true);
  For i := 0 To high(blocks) Do
    blocks[i] := Block.Create;

  // Set up the initial block
  reset();
End;

Destructor FractureDemo.Destroy;
Var
  i: Integer;
Begin
  ball.body.Free;
  ball.free;
  For i := 0 To high(blocks) Do
    blocks[i].free;

  Inherited Destroy();
End;

Procedure FractureDemo.generateContacts;
Var
  plane: CollisionPlane;
  i, j: Integer;
Begin
  hit := false;

  // Create the ground plane data
  plane := CollisionPlane.Create;
  plane.direction := V3(0, 1, 0);
  plane.offset := 0;

  // Set up the collision data structure
  cData.reset(maxContacts);
  cData.friction := 0.9;
  cData.restitution := 0.2;
  cData.tolerance := 0.1;

  // Perform collision detection
  For i := 0 To high(blocks) Do Begin

    If (Not blocks[i].exists) Then continue;

    // Check for collisions with the ground plane
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.boxAndHalfSpace(blocks[i], plane, cData);

    If (ball_active) Then Begin
      // And with the sphere
      If (Not cData.hasMoreContacts()) Then exit;
      If (CollisionDetector.boxAndSphere(blocks[i], ball, cData) <> 0) Then Begin
        hit := true;
        fracture_contact := cData.contactCount - 1;
      End;
    End;

    // Check for collisions with each other box
    For j := i + 1 To high(blocks) Do Begin

      If (Not blocks[j].exists) Then continue;

      If (Not cData.hasMoreContacts()) Then exit;
      CollisionDetector.boxAndBox(blocks[i], blocks[j], cData);
    End;
  End;

  // Check for sphere ground collisions
  If (ball_active) Then Begin
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.sphereAndHalfSpace(ball, plane, cData);
  End;
  plane.free;
End;

Procedure FractureDemo.updateObjects(duration: real);
Var
  i: Integer;
Begin

  For i := 0 To high(blocks) Do Begin
    If (blocks[i].exists) Then Begin
      blocks[i].body.integrate(duration);
      blocks[i].calculateInternals();
    End;
  End;

  If (ball_active) Then Begin
    ball.body.integrate(duration);
    ball.calculateInternals();
  End;
End;

Procedure FractureDemo.reset;
Var
  i: Integer;
  it: Matrix3;
Begin
  // Only the first block exists
  blocks[0].exists := true;
  For i := 1 To high(blocks) Do Begin
    blocks[i].exists := false;
  End;

  // Set the first block
  blocks[0].halfSize := V3(4, 4, 4);
  blocks[0].body.setPosition(0, 7, 0);
  blocks[0].body.setOrientation(1, 0, 0, 0);
  blocks[0].body.setVelocity(0, 0, 0);
  blocks[0].body.setRotation(0, 0, 0);
  blocks[0].body.setMass(100.0);

  it.setBlockInertiaTensor(blocks[0].halfSize, 100.0);
  blocks[0].body.setInertiaTensor(it);
  blocks[0].body.setDamping(0.9, 0.9);
  blocks[0].body.calculateDerivedData();
  blocks[0].calculateInternals();

  blocks[0].body.setAcceleration(GRAVITY);
  blocks[0].body.setAwake(true);
  blocks[0].body.setCanSleep(true);

  ball_active := true;

  // Set up the ball
  ball.body.setPosition(0, 5.0, 20.0);
  ball.body.setOrientation(1, 0, 0, 0);
  ball.body.setVelocity(
    random.randomBinomial(4.0),
    random.randomReal(1.0, 6.0),
    -20.0
    );
  ball.body.setRotation(0, 0, 0);
  ball.body.calculateDerivedData();
  ball.body.setAwake(true);
  ball.calculateInternals();

  hit := false;

  // Reset the contacts
  cData.contactCount := 0;
End;

Procedure FractureDemo.update;
Begin
  Inherited update();

  // Handle fractures.
  If (hit) Then Begin
    blocks[0].divideBlock(
      @cData.contactArray[fracture_contact],
      @blocks[0],
      @blocks[1]
      );
    ball_active := false;
  End;
End;

Function FractureDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Fracture Demo';
End;

Procedure FractureDemo.display;
Const
  lightPosition: Array Of single = (0.7, 1, 0.4, 0);
Var
  pos: Vector3;
  i, j: Integer;
  _theta: float;
Begin
  Inherited display();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glLightfv(GL_LIGHT0, GL_POSITION, @lightPosition[0]);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  glEnable(GL_NORMALIZE);
  For i := 0 To high(blocks) Do Begin
    If (blocks[i].exists) Then blocks[i].render();
  End;
  glDisable(GL_NORMALIZE);

  If (ball_active) Then Begin
    glColor3f(0.4, 0.7, 0.4);
    glPushMatrix();
    pos := ball.body.getPosition();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(0.25, 16, 8);
    glPopMatrix();
  End;

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

  drawDebug();
End;

End.

