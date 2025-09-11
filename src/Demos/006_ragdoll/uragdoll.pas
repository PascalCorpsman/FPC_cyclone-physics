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
  , urandom, ucollide_fine, ujoints //  ,  ucore, uprecision, uparticle, utiming, upcontacts
  ;

Const
  NUM_BONES = 12;
  NUM_JOINTS = 11;

Type

  Bone = Object(CollisionBox)

    //  public:
    //      Bone()
    //      {
    //          body = new cyclone::RigidBody();
    //      }
    //
    //      ~Bone()
    //      {
    //          delete body;
    //      }
    //
    //      /**
    //       * We use a sphere to collide bone on bone to allow some limited
    //       * interpenetration.
    //       */
    //      cyclone::CollisionSphere getCollisionSphere() const
    //      {
    //          cyclone::CollisionSphere sphere;
    //          sphere.body = body;
    //          sphere.radius = halfSize.x;
    //          sphere.offset = cyclone::Matrix4();
    //          if (halfSize.y < sphere.radius) sphere.radius = halfSize.y;
    //          if (halfSize.z < sphere.radius) sphere.radius = halfSize.z;
    //          sphere.calculateInternals();
    //          return sphere;
    //      }
    //
    //      /** Draws the bone. */
    //      void render()
    //      {
    //          // Get the OpenGL transformation
    //          GLfloat mat[16];
    //          body->getGLTransform(mat);
    //
    //          if (body->getAwake()) glColor3f(0.5f, 0.3f, 0.3f);
    //          else glColor3f(0.3f, 0.3f, 0.5f);
    //
    //          glPushMatrix();
    //          glMultMatrixf(mat);
    //          glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
    //          glutSolidCube(1.0f);
    //          glPopMatrix();
    //      }
    //
    //      /** Sets the bone to a specific location. */
    //      void setState(const cyclone::Vector3 &position,
    //                    const cyclone::Vector3 &extents)
    //      {
    //          body->setPosition(position);
    //          body->setOrientation(cyclone::Quaternion());
    //          body->setVelocity(cyclone::Vector3());
    //          body->setRotation(cyclone::Vector3());
    //          halfSize = extents;
    //
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
    //
    //          body->calculateDerivedData();
    //          calculateInternals();
    //      }
    //
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
    //    virtual void updateObjects(cyclone::real duration);
    //
    (** Resets the position of all the bones. *)
    Procedure reset(); override;

  public
    Constructor Create(); override;

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

{ RagdollDemo }

Procedure RagdollDemo.reset;
Begin
  Inherited reset();
End;

Constructor RagdollDemo.Create;
Begin
  Inherited Create();
  // Set up the bone hierarchy.

   // Right Knee
//    joints[0].set(
//        bones[0].body, cyclone::Vector3(0, 1.07f, 0),
//        bones[1].body, cyclone::Vector3(0, -1.07f, 0),
//        0.15f
//        );
//
//    // Left Knee
//    joints[1].set(
//        bones[2].body, cyclone::Vector3(0, 1.07f, 0),
//        bones[3].body, cyclone::Vector3(0, -1.07f, 0),
//        0.15f
//        );
//
//    // Right elbow
//    joints[2].set(
//        bones[9].body, cyclone::Vector3(0, 0.96f, 0),
//        bones[8].body, cyclone::Vector3(0, -0.96f, 0),
//        0.15f
//        );
//
//    // Left elbow
//    joints[3].set(
//        bones[11].body, cyclone::Vector3(0, 0.96f, 0),
//        bones[10].body, cyclone::Vector3(0, -0.96f, 0),
//        0.15f
//        );
//
//    // Stomach to Waist
//    joints[4].set(
//        bones[4].body, cyclone::Vector3(0.054f, 0.50f, 0),
//        bones[5].body, cyclone::Vector3(-0.043f, -0.45f, 0),
//        0.15f
//        );
//
//    joints[5].set(
//        bones[5].body, cyclone::Vector3(-0.043f, 0.411f, 0),
//        bones[6].body, cyclone::Vector3(0, -0.411f, 0),
//        0.15f
//        );
//
//    joints[6].set(
//        bones[6].body, cyclone::Vector3(0, 0.521f, 0),
//        bones[7].body, cyclone::Vector3(0, -0.752f, 0),
//        0.15f
//        );
//
//    // Right hip
//    joints[7].set(
//        bones[1].body, cyclone::Vector3(0, 1.066f, 0),
//        bones[4].body, cyclone::Vector3(0, -0.458f, -0.5f),
//        0.15f
//        );
//
//    // Left Hip
//    joints[8].set(
//        bones[3].body, cyclone::Vector3(0, 1.066f, 0),
//        bones[4].body, cyclone::Vector3(0, -0.458f, 0.5f),
//        0.105f
//        );
//
//    // Right shoulder
//    joints[9].set(
//        bones[6].body, cyclone::Vector3(0, 0.367f, -0.8f),
//        bones[8].body, cyclone::Vector3(0, 0.888f, 0.32f),
//        0.15f
//        );
//
//    // Left shoulder
//    joints[10].set(
//        bones[6].body, cyclone::Vector3(0, 0.367f, 0.8f),
//        bones[10].body, cyclone::Vector3(0, 0.888f, -0.32f),
//        0.15f
//        );

   // Set up the initial positions
  reset();

End;

Procedure RagdollDemo.initGraphics();
Begin
  Inherited initGraphics();
End;

Function RagdollDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Ragdoll Demo';
End;

Procedure RagdollDemo.display();
Begin
  Inherited display();
End;

End.

