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

  (**
   * Represents a primitive to detect collisions against.
   *)

  { CollisionPrimitive }

  CollisionPrimitive = Class
  public

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
    Function getTransform(): Matrix4;

  protected
    (**
     * The resultant transform of the primitive. This is
     * calculated by combining the offset of the primitive
     * with the transform of the rigid body.
     *)
    transform: Matrix4;
  public
    Constructor Create; virtual;
    Destructor Destroy; override;
  End;

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

  (**
   * A wrapper class that holds fast intersection tests. These
   * can be used to drive the coarse collision detection system or
   * as an early out in the full collision tests below.
   *)

  { IntersectionTests }

  IntersectionTests = Class
  public

    Class Function sphereAndHalfSpace(
      Const sphere: CollisionSphere;
      Const plane: CollisionPlane): Boolean;

    Class Function sphereAndSphere(
      Const one: CollisionSphere;
      Const two: CollisionSphere): Boolean;

    Class Function boxAndBox(
      Const one: CollisionBox;
      Const two: CollisionBox): Boolean;

    (**
     * Does an intersection test on an arbitrarily aligned box and a
     * half-space.
     *
     * The box is given as a transform matrix, including
     * position, and a vector of half-sizes for the extend of the
     * box along each local axis.
     *
     * The half-space is given as a direction (i.e. unit) vector and the
     * offset of the limiting plane from the origin, along the given
     * direction.
     *)
    Class Function boxAndHalfSpace(
      Const box: CollisionBox;
      Const plane: CollisionPlane): Boolean;
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

    Class Function boxAndBox(
      Const one: CollisionBox;
      Const two: CollisionBox;
      Var data: CollisionData
      ): unsigned;

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
  Const axis: Vector3): Float Inline;
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

Function CollisionPrimitive.getTransform: Matrix4;
Begin
  result := transform;
End;

{ IntersectionTests }

Class Function IntersectionTests.sphereAndHalfSpace(
  Const sphere: CollisionSphere; Const plane: CollisionPlane): Boolean;
Var
  ballDistance: float;
Begin
  // Find the distance from the origin
  ballDistance :=
    plane.direction *
    sphere.getAxis(3) -
    sphere.radius;

  // Check for the intersection
  result := ballDistance <= plane.offset;
End;

Class Function IntersectionTests.sphereAndSphere(Const one: CollisionSphere;
  Const two: CollisionSphere): Boolean;
Var
  midline: Vector3;
Begin
  // Find the vector between the objects
  midline := one.getAxis(3) - two.getAxis(3);

  // See if it is large enough.
  result := midline.squareMagnitude() <
    (one.radius + two.radius) * (one.radius + two.radius);
End;

(**
 * This function checks if the two boxes overlap
 * along the given axis. The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 *)

Function overlapOnAxis(
  Const one: CollisionBox;
  Const two: CollisionBox;
  Const axis: Vector3;
  Const toCentre: Vector3
  ): Boolean;
Var
  oneProject, twoProject, distance: Float;
Begin
  // Project the half-size of one onto axis
  oneProject := transformToAxis(one, axis);
  twoProject := transformToAxis(two, axis);

  // Project this onto the axis
  distance := real_abs(toCentre * axis);

  // Check for overlap
  result := (distance < oneProject + twoProject);
End;

Class Function IntersectionTests.boxAndBox(Const one: CollisionBox;
  Const two: CollisionBox): Boolean;

Var
  toCentre: Vector3;

  Function TEST_OVERLAP(Const axis: Vector3): boolean;
  Begin
    result := overlapOnAxis(one, two, (axis), toCentre);
  End;

Begin
  // Find the vector between the two centres
  toCentre := two.getAxis(3) - one.getAxis(3);

  result := (
    // Check on box one's axes first
    TEST_OVERLAP(one.getAxis(0))) And (
    TEST_OVERLAP(one.getAxis(1))) And (
    TEST_OVERLAP(one.getAxis(2))) And (

    // And on two's
    TEST_OVERLAP(two.getAxis(0))) And (
    TEST_OVERLAP(two.getAxis(1))) And (
    TEST_OVERLAP(two.getAxis(2))) And (

    // Now on the cross products
    TEST_OVERLAP(one.getAxis(0) Mod two.getAxis(0))) And (
    TEST_OVERLAP(one.getAxis(0) Mod two.getAxis(1))) And (
    TEST_OVERLAP(one.getAxis(0) Mod two.getAxis(2))) And (
    TEST_OVERLAP(one.getAxis(1) Mod two.getAxis(0))) And (
    TEST_OVERLAP(one.getAxis(1) Mod two.getAxis(1))) And (
    TEST_OVERLAP(one.getAxis(1) Mod two.getAxis(2))) And (
    TEST_OVERLAP(one.getAxis(2) Mod two.getAxis(0))) And (
    TEST_OVERLAP(one.getAxis(2) Mod two.getAxis(1))) And (
    TEST_OVERLAP(one.getAxis(2) Mod two.getAxis(2))
    );
End;

Class Function IntersectionTests.boxAndHalfSpace(Const box: CollisionBox;
  Const plane: CollisionPlane): Boolean;
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
  result := boxDistance <= plane.offset;
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

Const
  // Go through each combination of + and - for each half-size
  mults: Array[0..7, 0..2] Of float =
  ((1, 1, 1), (-1, 1, 1), (1, -1, 1), (-1, -1, 1),
    (1, 1, -1), (-1, 1, -1), (1, -1, -1), (-1, -1, -1));

Var
  contact: pContact;
  contactsUsed: unsigned;
  i: integer;
  vertexPos: Vector3;
  vertexDistance: float;
Begin
  result := 0;
  // Make sure we have contacts
  If (data.contactsLeft <= 0) Then exit;

  // Check for intersection
  If (Not IntersectionTests.boxAndHalfSpace(box, plane)) Then exit;

  // We have an intersection, so find the intersection points. We can make
  // do with only checking vertices. If the box is resting on a plane
  // or on an edge, it will be reported as four or two contact points.

  contact := data.contacts;
  contactsUsed := 0;

  For i := 0 To 7 Do Begin

    // Calculate the position of each vertex
    vertexPos.create(mults[i][0], mults[i][1], mults[i][2]);
    vertexPos.componentProductUpdate(box.halfSize);
    vertexPos := box.transform.transform(vertexPos);

    // Calculate the distance from the plane
    vertexDistance := vertexPos * plane.direction;

    // Compare this to the plane's distance
    If (vertexDistance <= plane.offset) Then Begin
      // Create the contact data.

      // The contact point is halfway between the vertex and the
      // plane - we multiply the direction by half the separation
      // distance and add the vertex location.
      contact^.contactPoint := plane.direction;
      contact^.contactPoint := contact^.contactPoint * (vertexDistance - plane.offset);
      contact^.contactPoint := contact^.contactPoint + vertexPos;
      contact^.contactNormal := plane.direction;
      contact^.penetration := plane.offset - vertexDistance;

      // Write the appropriate data
      contact^.setBodyData(@box.body, Nil,
        data.friction, data.restitution);

      // Move onto the next contact
      contact := contact + 1;
      contactsUsed := contactsUsed + 1;
      If (contactsUsed = data.contactsLeft) Then Begin
        // FIXME: müsste hier nicht noch ein "data.addContacts(contactsUsed);" kommen ?
        Raise exception.create('Juhu, testen ob die obige Zeile nicht doch rein muss oder nicht (wenn nicht av und Zeile löschen, sonst rein)');
        result := contactsUsed;
        exit;
      End;
    End;
  End;

  data.addContacts(contactsUsed);
  result := contactsUsed;
End;

(*
 * This function checks if the two boxes overlap
 * along the given axis, returning the ammount of overlap.
 * The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 *)

Function penetrationOnAxis(
  Const one: CollisionBox;
  Const two: CollisionBox;
  Const axis: Vector3;
  Const toCentre: Vector3
  ): Float;
Var
  oneProject, twoProject, distance: Float;
Begin
  // Project the half-size of one onto axis
  oneProject := transformToAxis(one, axis);
  twoProject := transformToAxis(two, axis);

  // Project this onto the axis
  distance := real_abs(toCentre * axis);

  // Return the overlap (i.e. positive indicates
  // overlap, negative indicates separation).
  result := oneProject + twoProject - distance;
End;

Function tryAxis(
  Const one: CollisionBox;
  Const two: CollisionBox;
  axis: Vector3;
  Const toCentre: Vector3;
  index: unsigned;

  // These values may be updated
  Var smallestPenetration: Float;
  Var smallestCase: unsigned
  ): Boolean;
Var
  penetration: Float;
Begin
  result := true;
  // Make sure we have a normalized axis, and don't check almost parallel axes
  If (axis.squareMagnitude() < 0.0001) Then exit;
  axis.Normalize();
  result := false;

  penetration := penetrationOnAxis(one, two, axis, toCentre);

  If (penetration < 0) Then exit;
  If (penetration < smallestPenetration) Then Begin
    smallestPenetration := penetration;
    smallestCase := index;
  End;
  result := true;
End;


Procedure fillPointFaceBoxBox(
  Const one: CollisionBox;
  Const two: CollisionBox;
  Const toCentre: Vector3;
  Var data: CollisionData;
  best: unsigned;
  pen: float
  );
Var
  contact: pContact;
  vertex, normal: Vector3;

Begin
  // This method is called when we know that a vertex from
  // box two is in contact with box one.

  contact := data.contacts;

  // We know which axis the collision is on (i.e. best),
  // but we need to work out which of the two faces on
  // this axis.
  normal := one.getAxis(best);
  If (one.getAxis(best) * toCentre > 0) Then Begin
    normal := normal * -1.0;
  End;

  // Work out which vertex of box two we're colliding with.
  // Using toCentre doesn't work!
  vertex := two.halfSize;
  If (two.getAxis(0) * normal < 0) Then vertex.x := -vertex.x;
  If (two.getAxis(1) * normal < 0) Then vertex.y := -vertex.y;
  If (two.getAxis(2) * normal < 0) Then vertex.z := -vertex.z;

  // Create the contact data
  contact^.contactNormal := normal;
  contact^.penetration := pen;
  contact^.contactPoint := two.getTransform() * vertex;
  contact^.setBodyData(@one.body, @two.body,
    data.friction, data.restitution);
End;


Function contactPoint(
  Const pOne: Vector3;
  Const dOne: Vector3;
  oneSize: Float;
  Const pTwo: Vector3;
  Const dTwo: Vector3;
  twoSize: Float;

  // If this is true, and the contact point is outside
  // the edge (in the case of an edge-face contact) then
  // we use one's midpoint, otherwise we use two's.
  useOne: Boolean): Vector3;
Var
  toSt, cOne, cTwo: Vector3;
  dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo: float;
  denom, mua, mub: float;
Begin

  smOne := dOne.squareMagnitude();
  smTwo := dTwo.squareMagnitude();
  dpOneTwo := dTwo * dOne;

  toSt := pOne - pTwo;
  dpStaOne := dOne * toSt;
  dpStaTwo := dTwo * toSt;

  denom := smOne * smTwo - dpOneTwo * dpOneTwo;

  // Zero denominator indicates parrallel lines
  If (real_abs(denom) < 0.0001) Then Begin
    If useOne Then
      result := pOne
    Else
      result := pTwo;
    exit;
  End;

  mua := (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
  mub := (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

  // If either of the edges has the nearest point out
  // of bounds, then the edges aren't crossed, we have
  // an edge-face contact. Our point is on the edge, which
  // we know from the useOne parameter.
  If (mua > oneSize) Or (
    mua < -oneSize) Or (
    mub > twoSize) Or (
    mub < -twoSize) Then Begin

    If useOne Then
      result := pOne
    Else
      result := pTwo;
  End
  Else Begin
    cOne := pOne + dOne * mua;
    cTwo := pTwo + dTwo * mub;

    result := cOne * 0.5 + cTwo * 0.5;
  End;
End;

Class Function CollisionDetector.boxAndBox(Const one: CollisionBox;
  Const two: CollisionBox; Var data: CollisionData): unsigned;
Var
  toCentre: Vector3;
  pen: float;
  best: unsigned;
  Function CHECK_OVERLAP(Const axis: vector3; Const index: unsigned): boolean;
  Begin
    result := (Not tryAxis(one, two, (axis), toCentre, (index), pen, best));
  End;

Var
  bestSingleAxis, oneAxisIndex, twoAxisIndex, i: unsigned;
  vertex, ptOnOneEdge, ptOnTwoEdge, oneAxis, twoAxis, axis: Vector3;
  contact: pContact;
Begin
  result := 0;

  //if (!IntersectionTests::boxAndBox(one, two)) return 0;

  // Find the vector between the two centres
  toCentre := two.getAxis(3) - one.getAxis(3);

  // We start assuming there is no contact
  pen := REAL_MAX;
  best := $FFFFFF;

  // Now we check each axes, returning if it gives us
  // a separating axis, and keeping track of the axis with
  // the smallest penetration otherwise.
  If CHECK_OVERLAP(one.getAxis(0), 0) Then exit;
  If CHECK_OVERLAP(one.getAxis(1), 1) Then exit;
  If CHECK_OVERLAP(one.getAxis(2), 2) Then exit;

  If CHECK_OVERLAP(two.getAxis(0), 3) Then exit;
  If CHECK_OVERLAP(two.getAxis(1), 4) Then exit;
  If CHECK_OVERLAP(two.getAxis(2), 5) Then exit;

  // Store the best axis-major, in case we run into almost
  // parallel edge collisions later
  bestSingleAxis := best;

  If CHECK_OVERLAP(one.getAxis(0) Mod two.getAxis(0), 6) Then exit;
  If CHECK_OVERLAP(one.getAxis(0) Mod two.getAxis(1), 7) Then exit;
  If CHECK_OVERLAP(one.getAxis(0) Mod two.getAxis(2), 8) Then exit;
  If CHECK_OVERLAP(one.getAxis(1) Mod two.getAxis(0), 9) Then exit;
  If CHECK_OVERLAP(one.getAxis(1) Mod two.getAxis(1), 10) Then exit;
  If CHECK_OVERLAP(one.getAxis(1) Mod two.getAxis(2), 11) Then exit;
  If CHECK_OVERLAP(one.getAxis(2) Mod two.getAxis(0), 12) Then exit;
  If CHECK_OVERLAP(one.getAxis(2) Mod two.getAxis(1), 13) Then exit;
  If CHECK_OVERLAP(one.getAxis(2) Mod two.getAxis(2), 14) Then exit;

  // Make sure we've got a result.
  assert(best <> $FFFFFF);

  // We now know there's a collision, and we know which
  // of the axes gave the smallest penetration. We now
  // can deal with it in different ways depending on
  // the case.
  If (best < 3) Then Begin

    // We've got a vertex of box two on a face of box one.
    fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
    data.addContacts(1);
    result := 1;
  End
  Else If (best < 6) Then Begin
    // We've got a vertex of box one on a face of box two.
    // We use the same algorithm as above, but swap around
    // one and two (and therefore also the vector between their
    // centres).
    fillPointFaceBoxBox(two, one, toCentre * -1.0, data, best - 3, pen);
    data.addContacts(1);
    result := 1;
  End
  Else Begin
    // We've got an edge-edge contact. Find out which axes
    best := best - 6;
    oneAxisIndex := best Div 3;
    twoAxisIndex := best Mod 3;
    oneAxis := one.getAxis(oneAxisIndex);
    twoAxis := two.getAxis(twoAxisIndex);
    axis := oneAxis Mod twoAxis;
    axis.Normalize();

    // The axis should point from box one to box two.
    If (axis * toCentre > 0) Then axis := axis * -1.0;

    // We have the axes, but not the edges: each axis has 4 edges parallel
    // to it, we need to find which of the 4 for each object. We do
    // that by finding the point in the centre of the edge. We know
    // its component in the direction of the box's collision axis is zero
    // (its a mid-point) and we determine which of the extremes in each
    // of the other axes is closest.
    ptOnOneEdge := one.halfSize;
    ptOnTwoEdge := two.halfSize;

    For i := 0 To 2 Do Begin
      If (i = oneAxisIndex) Then
        ptOnOneEdge[i] := 0
      Else If (one.getAxis(i) * axis > 0) Then
        ptOnOneEdge[i] := -ptOnOneEdge[i];

      If (i = twoAxisIndex) Then
        ptOnTwoEdge[i] := 0
      Else If (two.getAxis(i) * axis < 0) Then
        ptOnTwoEdge[i] := -ptOnTwoEdge[i];
    End;

    // Move them into world coordinates (they are already oriented
    // correctly, since they have been derived from the axes).
    ptOnOneEdge := one.transform * ptOnOneEdge;
    ptOnTwoEdge := two.transform * ptOnTwoEdge;

    // So we have a point and a direction for the colliding edges.
    // We need to find out point of closest approach of the two
    // line-segments.
    vertex := contactPoint(
      ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
      ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
      bestSingleAxis > 2
      );

    // We can fill the contact.
    contact := data.contacts;

    contact^.penetration := pen;
    contact^.contactNormal := axis;
    contact^.contactPoint := vertex;
    contact^.setBodyData(@one.body, @two.body,
      data.friction, data.restitution);
    data.addContacts(1);
    result := 1;
  End;
End;

End.

