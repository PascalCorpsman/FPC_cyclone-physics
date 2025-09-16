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

    //    /** Processes the contact generation code. */
    //    virtual void generateContacts();
    //
    //    /** Processes the objects in the simulation forward in time. */
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
Begin
  body.setPosition(position);
  //          body->setOrientation(cyclone::Quaternion());
  //          body->setVelocity(cyclone::Vector3());
  //          body->setRotation(cyclone::Vector3());
  halfSize := extents;

  //          cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
  //          body->setMass(mass);
  //
  //          cyclone::Matrix3 tensor;
  //          tensor.setBlockInertiaTensor(halfSize, mass);
  //          body->setInertiaTensor(tensor);
  //
  //          body->setLinearDamping(0.95f);
  //          body->setAngularDamping(0.8f);
  //          body->clearAccumulators();
  //          body->setAcceleration(cyclone::Vector3::GRAVITY);
  //
  //          body->setCanSleep(false);
  //          body->setAwake();

  body.calculateDerivedData();
  calculateInternals();
End;

{ RagdollDemo }

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

  //    cyclone::real strength = -random.randomReal(500.0f, 1000.0f);
  //    for (unsigned i = 0; i < NUM_BONES; i++)
  //    {
  //        bones[i].body->addForceAtBodyPoint(
  //            cyclone::Vector3(strength, 0, 0), cyclone::Vector3()
  //            );
  //    }
  //    bones[6].body->addForceAtBodyPoint(
  //        cyclone::Vector3(strength, 0, random.randomBinomial(1000.0f)),
  //        cyclone::Vector3(random.randomBinomial(4.0f), random.randomBinomial(3.0f), 0)
  //        );
  //
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
  For i := 0 To high(bones) Do
    bones[i] := bone.Create;

  // Right Knee
  joints[0]._Set(
    bones[0].body, V3(0, 1.07, 0),
    bones[1].body, V3(0, -1.07, 0),
    0.15
    );

  // Left Knee
  joints[1]._set(
    bones[2].body, V3(0, 1.07, 0),
    bones[3].body, V3(0, -1.07, 0),
    0.15
    );

  // Right elbow
  joints[2]._set(
    bones[9].body, V3(0, 0.96, 0),
    bones[8].body, V3(0, -0.96, 0),
    0.15
    );

  // Left elbow
  joints[3]._set(
    bones[11].body, V3(0, 0.96, 0),
    bones[10].body, V3(0, -0.96, 0),
    0.15
    );

  // Stomach to Waist
  joints[4]._set(
    bones[4].body, V3(0.054, 0.50, 0),
    bones[5].body, V3(-0.043, -0.45, 0),
    0.15
    );

  joints[5]._set(
    bones[5].body, V3(-0.043, 0.411, 0),
    bones[6].body, V3(0, -0.411, 0),
    0.15
    );

  joints[6]._set(
    bones[6].body, V3(0, 0.521, 0),
    bones[7].body, V3(0, -0.752, 0),
    0.15
    );

  // Right hip
  joints[7]._set(
    bones[1].body, V3(0, 1.066, 0),
    bones[4].body, V3(0, -0.458, -0.5),
    0.15
    );

  // Left Hip
  joints[8]._set(
    bones[3].body, V3(0, 1.066, 0),
    bones[4].body, V3(0, -0.458, 0.5),
    0.105
    );

  // Right shoulder
  joints[9]._set(
    bones[6].body, V3(0, 0.367, -0.8),
    bones[8].body, V3(0, 0.888, 0.32),
    0.15
    );

  // Left shoulder
  joints[10]._set(
    bones[6].body, V3(0, 0.367, 0.8),
    bones[10].body, V3(0, 0.888, -0.32),
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
  For i := 0 To high(bones) Do
    bones[i].Free;
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
    a_pos := _joint.body[0].getPointInWorldSpace(@_joint.position[0]);
    b_pos := _joint.body[1].getPointInWorldSpace(@_joint.position[1]);
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

