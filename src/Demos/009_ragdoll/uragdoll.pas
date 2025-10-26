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
Unit uragdoll;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp
  , urandom, ucollide_fine, ujoints, ucore, ubody, uprecision //, uparticle, utiming, upcontacts
  ;

Const
  NUM_BONES = 12;
  NUM_JOINTS = 11;

Type

  { Bone }

  Bone = Class(CollisionBox)
  public
    (**
     * We use a sphere to collide bone on bone to allow some limited
     * interpenetration.
     *)
    Function getCollisionSphere(): CollisionSphere;

    (** Draws the bone. *)
    Procedure render();

    (** Sets the bone to a specific location. *)
    Procedure setState(Const position, extents: Vector3);
  End;

  { RagdollDemo }

  RagdollDemo = Class(RigidBodyApplication)
  private
  protected

    (** Holds the bone bodies. *)
    bones: Array[0..NUM_BONES - 1] Of Bone;

    //    /** Holds the joints. */
    joints: Array[0..NUM_JOINTS - 1] Of Joint;

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
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := RagdollDemo.Create;
End;

{ Bone }

Function Bone.getCollisionSphere(): CollisionSphere;
Var
  sphere: CollisionSphere;
Begin
  sphere := CollisionSphere.Create;
  sphere.body := body;
  sphere.radius := halfSize.x;
  sphere.offset.create();
  If (halfSize.y < sphere.radius) Then sphere.radius := halfSize.y;
  If (halfSize.z < sphere.radius) Then sphere.radius := halfSize.z;
  sphere.calculateInternals();
  result := sphere;
End;

Procedure Bone.render;
Var
  mat: TOpenGLMatrix;
Begin
  // Get the OpenGL transformation
  body.getGLTransform(mat);

  If (body.getAwake()) Then
    glColor3f(0.5, 0.3, 0.3)
  Else
    glColor3f(0.3, 0.3, 0.5);

  glPushMatrix();
  glMultMatrixf(@mat[0]);
  glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
  glutSolidCube(1.0);
  glPopMatrix();
End;

Procedure Bone.setState(Const position, extents: Vector3);
Var
  q: Quaternion;
  v: Vector3;
  mass: float;
  tensor: Matrix3;
Begin
  body.setPosition(position);
  q.Create;
  body.setOrientation(Q);
  v.create;
  body.setVelocity(V);
  body.setRotation(V);
  halfSize := extents;

  mass := halfSize.x * halfSize.y * halfSize.z * 8.0;
  body.setMass(mass);

  tensor.setBlockInertiaTensor(halfSize, mass);
  body.setInertiaTensor(tensor);

  body.setLinearDamping(0.95);
  body.setAngularDamping(0.8);
  body.clearAccumulators();
  body.setAcceleration(GRAVITY);

  body.setCanSleep(false);
  body.setAwake();

  body.calculateDerivedData();
  calculateInternals();
End;

{ RagdollDemo }

Procedure RagdollDemo.generateContacts();
Var
  plane: CollisionPlane;
  bone_, other: ^Bone;
  i, j: integer;
  boneSphere, otherSphere: CollisionSphere;
  joint_: ^joint;
  added: unsigned;
Begin
  // Create the ground plane data
  plane := CollisionPlane.Create;
  plane.direction := V3(0, 1, 0);
  plane.offset := 0;

  // Set up the collision data structure
  cData.reset(maxContacts);
  cData.friction := 0.9;
  cData.restitution := 0.6;
  cData.tolerance := 0.1;

  // Perform exhaustive collision detection on the ground plane
  For i := 0 To NUM_BONES - 1 Do Begin
    //    for (Bone *bone = bones; bone < bones+NUM_BONES; bone++)
    bone_ := @bones[i];
    // Check for collisions with the ground plane
    If (Not cData.hasMoreContacts()) Then exit;
    CollisionDetector.boxAndHalfSpace(bone_^, plane, cdata);

    boneSphere := bone_^.getCollisionSphere();

    // Check for collisions with each other box
    For j := i + 1 To NUM_BONES - 1 Do Begin
      //        for (Bone *other = bone+1; other < bones+NUM_BONES; other++)
      other := @bones[j];
      If (Not cData.hasMoreContacts()) Then Begin
        boneSphere.free;
        exit;
      End;

      otherSphere := other^.getCollisionSphere();

      CollisionDetector.sphereAndSphere(
        boneSphere,
        otherSphere,
        cData
        );
    End;
    boneSphere.free;
  End;

  // Check for joint violation
  For i := 0 To NUM_JOINTS - 1 Do Begin
    joint_ := @joints[i];
    If (Not cData.hasMoreContacts()) Then exit;
    added := joint_^.addContact(cData.contacts, cData.contactsLeft);
    cData.addContacts(added);
  End;
  plane.free;
End;

Procedure RagdollDemo.updateObjects(duration: real);
Var
  i: integer;
Begin
  For i := 0 To high(bones) Do Begin
    bones[i].body.integrate(duration);
    bones[i].calculateInternals();
  End;
End;

Procedure RagdollDemo.reset;
Var
  strength: Float;
  i: integer;
Begin
  bones[0].setState(
    V3(0, 0.993, -0.5),
    V3(0.301, 1.0, 0.234));
  bones[1].setState(
    V3(0, 3.159, -0.56),
    V3(0.301, 1.0, 0.234));
  bones[2].setState(
    V3(0, 0.993, 0.5),
    V3(0.301, 1.0, 0.234));
  bones[3].setState(
    V3(0, 3.15, 0.56),
    V3(0.301, 1.0, 0.234));
  bones[4].setState(
    V3(-0.054, 4.683, 0.013),
    V3(0.415, 0.392, 0.690));
  bones[5].setState(
    V3(0.043, 5.603, 0.013),
    V3(0.301, 0.367, 0.693));
  bones[6].setState(
    V3(0, 6.485, 0.013),
    V3(0.435, 0.367, 0.786));
  bones[7].setState(
    V3(0, 7.759, 0.013),
    V3(0.45, 0.598, 0.421));
  bones[8].setState(
    V3(0, 5.946, -1.066),
    V3(0.267, 0.888, 0.207));
  bones[9].setState(
    V3(0, 4.024, -1.066),
    V3(0.267, 0.888, 0.207));
  bones[10].setState(
    V3(0, 5.946, 1.066),
    V3(0.267, 0.888, 0.207));
  bones[11].setState(
    V3(0, 4.024, 1.066),
    V3(0.267, 0.888, 0.207));

  strength := -random.randomReal(500.0, 1000.0);

  (* -- Debug forces are removed -> Radgoll will fall to floor ..
  For i := 0 To NUM_BONES - 1 Do Begin
    bones[i].body.addForceAtBodyPoint(
      V3(strength, 0, 0), V3(0, 0, 0));
  End;

  bones[6].body.addForceAtBodyPoint(
    V3(strength, 0, random.randomBinomial(1000.0)),
    V3(random.randomBinomial(4.0), random.randomBinomial(3.0), 0)
    );
  // *)

  // Reset the contacts
  cData.contactCount := 0;
End;

Constructor RagdollDemo.Create;
Var
  i: Integer;
Begin
  Inherited Create();
  // Set up the bone hierarchy.
  For i := 0 To high(joints) Do
    joints[i] := Joint.Create;
  For i := 0 To high(bones) Do Begin
    bones[i] := bone.Create;
    bones[i].body := RigidBody.Create();
  End;

  // Right Knee
  joints[0]._Set(
    @bones[0].body, V3(0, 1.07, 0),
    @bones[1].body, V3(0, -1.07, 0),
    0.15
    );

  // Left Knee
  joints[1]._set(
    @bones[2].body, V3(0, 1.07, 0),
    @bones[3].body, V3(0, -1.07, 0),
    0.15
    );

  // Right elbow
  joints[2]._set(
    @bones[9].body, V3(0, 0.96, 0),
    @bones[8].body, V3(0, -0.96, 0),
    0.15
    );

  // Left elbow
  joints[3]._set(
    @bones[11].body, V3(0, 0.96, 0),
    @bones[10].body, V3(0, -0.96, 0),
    0.15
    );

  // Stomach to Waist
  joints[4]._set(
    @bones[4].body, V3(0.054, 0.50, 0),
    @bones[5].body, V3(-0.043, -0.45, 0),
    0.15
    );

  joints[5]._set(
    @bones[5].body, V3(-0.043, 0.411, 0),
    @bones[6].body, V3(0, -0.411, 0),
    0.15
    );

  joints[6]._set(
    @bones[6].body, V3(0, 0.521, 0),
    @bones[7].body, V3(0, -0.752, 0),
    0.15
    );

  // Right hip
  joints[7]._set(
    @bones[1].body, V3(0, 1.066, 0),
    @bones[4].body, V3(0, -0.458, -0.5),
    0.15
    );

  // Left Hip
  joints[8]._set(
    @bones[3].body, V3(0, 1.066, 0),
    @bones[4].body, V3(0, -0.458, 0.5),
    0.105
    );

  // Right shoulder
  joints[9]._set(
    @bones[6].body, V3(0, 0.367, -0.8),
    @bones[8].body, V3(0, 0.888, 0.32),
    0.15
    );

  // Left shoulder
  joints[10]._set(
    @bones[6].body, V3(0, 0.367, 0.8),
    @bones[10].body, V3(0, 0.888, -0.32),
    0.15
    );

  // Set up the initial positions
  reset();
End;

Destructor RagdollDemo.Destroy;
Var
  i: Integer;
Begin
  For i := 0 To high(Joints) Do
    joints[i].Free;
  For i := 0 To high(bones) Do Begin
    bones[i].body.Free;
    bones[i].Free;
  End;
  Inherited Destroy();
End;

Procedure RagdollDemo.initGraphics;
Const
  lightAmbient: Array[0..3] Of single = (0.8, 0.8, 0.8, 1.0);
  lightDiffuse: Array[0..3] Of single = (0.9, 0.95, 1.0, 1.0);
Begin
  glLightfv(GL_LIGHT0, GL_AMBIENT, @lightAmbient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, @lightDiffuse);
  glEnable(GL_LIGHT0);
  Inherited initGraphics();
End;

Function RagdollDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Ragdoll Demo';
End;

Procedure RagdollDemo.display;
Const
  lightPosition: Array[0..3] Of single = (0.7, -1, 0.4, 0);
  lightPositionMirror: Array[0..3] Of single = (0.7, 1, 0.4, 0);
Var
  i, j: Integer;
  _Joint: Joint;
  a_pos, b_pos: Vector3;
  _theta, _length: Float;
Begin

  Inherited display();

  // Render the bones
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  glLightfv(GL_LIGHT0, GL_POSITION, @lightPosition);
  glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);

  glEnable(GL_NORMALIZE);
  glColor3f(1, 0, 0);
  For i := 0 To NUM_BONES - 1 Do Begin
    bones[i].render();
  End;
  glDisable(GL_NORMALIZE);

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);

  glDisable(GL_DEPTH_TEST);
  glBegin(GL_LINES);
  For i := 0 To NUM_JOINTS - 1 Do Begin
    _Joint := joints[i];
    a_pos := _joint.body[0]^.getPointInWorldSpace(@_joint.position[0]);
    b_pos := _joint.body[1]^.getPointInWorldSpace(@_joint.position[1]);
    _length := (b_pos - a_pos).magnitude();

    If (_length > _joint.error) Then
      glColor3f(1, 0, 0)
    Else
      glColor3f(0, 1, 0);

    glVertex3f(a_pos.x, a_pos.y, a_pos.z);
    glVertex3f(b_pos.x, b_pos.y, b_pos.z);
  End;
  glEnd();
  glEnable(GL_DEPTH_TEST);

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

