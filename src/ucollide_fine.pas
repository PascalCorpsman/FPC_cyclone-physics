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
Unit ucollide_fine;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucontacts, ucore, ubody, uprecision;

Type


  // Forward declarations of primitive friends
  //    class IntersectionTests;
  //    class CollisionDetector;

  (**
   * Represents a primitive to detect collisions against.
   *)

  { CollisionPrimitive }

  CollisionPrimitive = Class
  public
    //        /**
    //         * This class exists to help the collision detector
    //         * and intersection routines, so they should have
    //         * access to its data.
    //         */
    //        friend class IntersectionTests;
    //        friend class CollisionDetector;

            (**
             * The rigid body that is represented by this primitive.
             *)
    body: RigidBody;

    (**
     * The offset of this primitive from the given rigid body.
     *)
    offset: Matrix4;

    (**
     * Calculates the internals for the primitive.
     *)
    Procedure calculateInternals();

    (**
     * This is a convenience function to allow access to the
     * axis vectors in the transform for this primitive.
     *)
    Function getAxis(index: unsigned): Vector3;

    (**
     * Returns the resultant transform of the primitive, calculated from
     * the combined offset of the primitive and the transform
     * (orientation + position) of the rigid body to which it is
     * attached.
     *)
    Function getTransform(): PMatrix4;

  protected
    (**
     * The resultant transform of the primitive. This is
     * calculated by combining the offset of the primitive
     * with the transform of the rigid body.
     *)
    transform: Matrix4;
  public
    Constructor Create; virtual;
    Destructor Destroy; virtual;
  End;

  //    /**
  //     * Represents a rigid body that can be treated as a sphere
  //     * for collision detection.
  //     */
  //    class CollisionSphere : public CollisionPrimitive
  //    {
  //    public:
  //        /**
  //         * The radius of the sphere.
  //         */
  //        real radius;
  //    };

      (**
       * The plane is not a primitive: it doesn't represent another
       * rigid body. It is used for contacts with the immovable
       * world geometry.
       *)
  CollisionPlane = Class
  public
    (**
     * The plane normal
     *)
    direction: Vector3;

    (**
     * The distance of the plane from the origin.
     *)
    offset: float;
  End;

  (**
   * Represents a rigid body that can be treated as an aligned bounding
   * box for collision detection.
   *)

  CollisionBox = Class(CollisionPrimitive)
  public
    (**
     * Holds the half-sizes of the box along each of its local axes.
     *)
    halfSize: Vector3;
  End;
  //
  //    /**
  //     * A wrapper class that holds fast intersection tests. These
  //     * can be used to drive the coarse collision detection system or
  //     * as an early out in the full collision tests below.
  //     */
  //    class IntersectionTests
  //    {
  //    public:
  //
  //        static bool sphereAndHalfSpace(
  //            const CollisionSphere &sphere,
  //            const CollisionPlane &plane);
  //
  //        static bool sphereAndSphere(
  //            const CollisionSphere &one,
  //            const CollisionSphere &two);
  //
  //        static bool boxAndBox(
  //            const CollisionBox &one,
  //            const CollisionBox &two);
  //
  //        /**
  //         * Does an intersection test on an arbitrarily aligned box and a
  //         * half-space.
  //         *
  //         * The box is given as a transform matrix, including
  //         * position, and a vector of half-sizes for the extend of the
  //         * box along each local axis.
  //         *
  //         * The half-space is given as a direction (i.e. unit) vector and the
  //         * offset of the limiting plane from the origin, along the given
  //         * direction.
  //         */
  //        static bool boxAndHalfSpace(
  //            const CollisionBox &box,
  //            const CollisionPlane &plane);
  //    };

    (**
     * Represents a rigid body that can be treated as a sphere
     * for collision detection.
     *)
  CollisionSphere = Class(CollisionPrimitive)
  public
    (**
     * The radius of the sphere.
     *)
    radius: float;
  End;

  (**
   * A helper structure that contains information for the detector to use
   * in building its contact data.
   *)

  { CollisionData }

  CollisionData = Object
    (**
     * Holds the base of the collision data: the first contact
     * in the array. This is used so that the contact pointer (below)
     * can be incremented each time a contact is detected, while
     * this pointer points to the first contact found.
     *)
    contactArray: pContact;

    (** Holds the contact array to write into. *)
    contacts: PContact;

    (** Holds the maximum number of contacts the array can take. *)
    contactsLeft: Integer;

    (** Holds the number of contacts found so far. *)
    contactCount: unsigned;

    (** Holds the friction value to write into any collisions. *)
    friction: float;

    (** Holds the restitution value to write into any collisions. *)
    restitution: float;

    (**
     * Holds the collision tolerance, even uncolliding objects this
     * close should have collisions generated.
     *)
    tolerance: float;

    (**
     * Checks if there are more contacts available in the contact
     * data.
     *)
    Function hasMoreContacts(): Boolean;

    (**
     * Resets the data so that it has no used contacts recorded.
     *)
    Procedure reset(maxContacts: unsigned);

    (**
     * Notifies the data that the given number of contacts have
     * been added.
     *)
    Procedure addContacts(count: unsigned);
  End;

  (**
    * A wrapper class that holds the fine grained collision detection
    * routines.
    *
    * Each of the functions has the same format: it takes the details
    * of two objects, and a pointer to a contact array to fill. It
    * returns the number of contacts it wrote into the array.
    *)

  { CollisionDetector }

  CollisionDetector = Class
  public
    //
    //        static unsigned sphereAndHalfSpace(
    //            const CollisionSphere &sphere,
    //            const CollisionPlane &plane,
    //            CollisionData *data
    //            );
    //
    //        static unsigned sphereAndTruePlane(
    //            const CollisionSphere &sphere,
    //            const CollisionPlane &plane,
    //            CollisionData *data
    //            );

    Class Function sphereAndSphere(
      Const one: CollisionSphere;
      Const two: CollisionSphere;
      Var data: CollisionData
      ): unsigned;

    (**
     * Does a collision test on a collision box and a plane representing
     * a half-space (i.e. the normal of the plane
     * points out of the half-space).
     *)
    Class Function boxAndHalfSpace(
      Const box: CollisionBox;
      Const plane: CollisionPlane;
      Var data: CollisionData
      ): unsigned;

    //        static unsigned boxAndBox(
    //            const CollisionBox &one,
    //            const CollisionBox &two,
    //            CollisionData *data
    //            );
    //
    //        static unsigned boxAndPoint(
    //            const CollisionBox &box,
    //            const Vector3 &point,
    //            CollisionData *data
    //            );
    //
    //        static unsigned boxAndSphere(
    //            const CollisionBox &box,
    //            const CollisionSphere &sphere,
    //            CollisionData *data
    //            );
  End;


Implementation

Function transformToAxis(
  Const box: CollisionBox;
  Const axis: Vector3): Float;
Begin
  result :=
    box.halfSize.x * real_abs(axis * box.getAxis(0)) +
    box.halfSize.y * real_abs(axis * box.getAxis(1)) +
    box.halfSize.z * real_abs(axis * box.getAxis(2));
End;



{ CollisionPrimitive }

Constructor CollisionPrimitive.Create;
Begin
  Inherited create;
  body := RigidBody.create();
  offset.create();
End;

Destructor CollisionPrimitive.Destroy;
Begin
  body.Free;
End;

Procedure CollisionPrimitive.calculateInternals;
Begin
  transform := body.getTransform() * offset;
End;

Function CollisionPrimitive.getAxis(index: unsigned): Vector3;
Begin
  result := transform.getAxisVector(index);
End;

Function CollisionPrimitive.getTransform: PMatrix4;
Begin
  result := @transform;
End;

Function CollisionData.hasMoreContacts: Boolean;
Begin
  result := contactsLeft > 0;
End;

Procedure CollisionData.reset(maxContacts: unsigned);
Begin
  contactsLeft := maxContacts;
  contactCount := 0;
  contacts := contactArray;
End;

Procedure CollisionData.addContacts(count: unsigned);
Begin
  // Reduce the number of contacts remaining, add number used
  contactsLeft := contactsLeft - count;
  contactCount := contactCount + count;

  // Move the array forward
  inc(contacts, count);
End;

{ CollisionDetector }

Class Function CollisionDetector.sphereAndSphere(Const one: CollisionSphere;
  Const two: CollisionSphere; Var data: CollisionData): unsigned;
Var
  normal, positionOne, positionTwo, midline: Vector3;
  size_: float;
  contact: pContact;
Begin
  result := 0;
  // Make sure we have contacts
  If (data.contactsLeft <= 0) Then exit;

  // Cache the sphere positions
  positionOne := one.getAxis(3);
  positionTwo := two.getAxis(3);

  // Find the vector between the objects
  midline := positionOne - positionTwo;
  size_ := midline.magnitude();

  // See if it is large enough.
  If (size_ <= 0.0) Or (size_ >= one.radius + two.radius) Then exit;

  // We manually create the normal, because we have the
  // size to hand.
  normal := midline * (1.0 / size_);

  contact := data.contacts;
  contact^.contactNormal := normal;
  contact^.contactPoint := positionOne + midline * 0.5;
  contact^.penetration := (one.radius + two.radius - size_);
  contact^.setBodyData(@one.body, @two.body,
    data.friction, data.restitution);

  data.addContacts(1);
  result := 1;
End;

Class Function CollisionDetector.boxAndHalfSpace(Const box: CollisionBox;
  Const plane: CollisionPlane; Var data: CollisionData): unsigned;
Var
  projectedRadius: Float;
  boxDistance: Float;
Begin
  // Work out the projected radius of the box onto the plane direction
  projectedRadius := transformToAxis(box, plane.direction);

  // Work out how far the box is from the origin
  boxDistance :=
    plane.direction *
    box.getAxis(3) -
    projectedRadius;

  // Check for the intersection
  result := ord(boxDistance <= plane.offset);
End;

End.

