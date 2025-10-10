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
Unit ucontacts;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uprecision, ubody, ucore;

Type

  (*
   * Forward declaration, see full declaration below for complete
   * documentation.
   *)
  ContactResolver = Class;

  (**
   * A contact represents two bodies in contact. Resolving a
   * contact removes their interpenetration, and applies sufficient
   * impulse to keep them apart. Colliding bodies may also rebound.
   * Contacts can be used to represent positional joints, by making
   * the contact constraint keep the bodies in their correct
   * orientation.
   *
   * It can be a good idea to create a contact object even when the
   * contact isn't violated. Because resolving one contact can violate
   * another, contacts that are close to being violated should be
   * sent to the resolver; that way if one resolution moves the body,
   * the contact may be violated, and can be resolved. If the contact
   * is not violated, it will not be resolved, so you only loose a
   * small amount of execution time.
   *
   * The contact has no callable functions, it just holds the contact
   * details. To resolve a set of contacts, use the contact resolver
   * class.
   *)

  { Contact }

  Contact = Object
    // ... Other data as before ...

//        /**
//         * The contact resolver object needs access into the contacts to
//         * set and effect the contact.
//         */
//        friend class ContactResolver;
//
  public
    (**
     * Holds the bodies that are involved in the contact. The
     * second of these can be NULL, for contacts with the scenery.
     *)
    body: Array[0..1] Of PRigidBody;

    (**
     * Holds the lateral friction coefficient at the contact.
     *)
    friction: float;

    (**
     * Holds the normal restitution coefficient at the contact.
     *)
    restitution: float;

    (**
     * Holds the position of the contact in world coordinates.
     *)
    contactPoint: Vector3;

    (**
     * Holds the direction of the contact in world coordinates.
     *)
    contactNormal: Vector3;

    (**
     * Holds the depth of penetration at the contact point. If both
     * bodies are specified then the contact point should be midway
     * between the inter-penetrating points.
     *)
    penetration: float;

    (**
     * Sets the data that doesn't normally depend on the position
     * of the contact (i.e. the bodies, and their material properties).
     *)
    Procedure setBodyData(one, two: PRigidBody;
      afriction, arestitution: Float);

  protected

    (**
     * A transform matrix that converts co-ordinates in the contact's
     * frame of reference to world co-ordinates. The columns of this
     * matrix form an orthonormal set of vectors.
     *)
    contactToWorld: Matrix3;

    (**
     * Holds the closing velocity at the point of contact. This is set
     * when the calculateInternals function is run.
     *)
    contactVelocity: Vector3;

    (**
     * Holds the required change in velocity for this contact to be
     * resolved.
     *)
    desiredDeltaVelocity: float;

    (**
     * Holds the world space position of the contact point relative to
     * centre of each body. This is set when the calculateInternals
     * function is run.
     *)
    relativeContactPosition: Array[0..1] Of Vector3;

  protected
    (**
     * Calculates internal data from state data. This is called before
     * the resolution algorithm tries to do any resolution. It should
     * never need to be called manually.
     *)
    Procedure calculateInternals(duration: Float);

    (**
     * Reverses the contact. This involves swapping the two rigid bodies
     * and reversing the contact normal. The internal values should then
     * be recalculated using calculateInternals (this is not done
     * automatically).
     *)
    Procedure swapBodies();

    (**
     * Updates the awake state of rigid bodies that are taking
     * place in the given contact. A body will be made awake if it
     * is in contact with a body that is awake.
     *)
    Procedure matchAwakeState();

    (**
     * Calculates and sets the internal value for the desired delta
     * velocity.
     *)
    Procedure calculateDesiredDeltaVelocity(duration: float);

    (**
     * Calculates and returns the velocity of the contact
     * point on the given body.
     *)
    Function calculateLocalVelocity(bodyIndex: unsigned; duration: Float): Vector3;

    (**
     * Calculates an orthonormal basis for the contact point, based on
     * the primary friction direction (for anisotropic friction) or
     * a random orientation (for isotropic friction).
     *)
    Procedure calculateContactBasis();

    //        /**
    //         * Applies an impulse to the given body, returning the
    //         * change in velocities.
    //         */
    //        void applyImpulse(const Vector3 &impulse, RigidBody *body,
    //                          Vector3 *velocityChange, Vector3 *rotationChange);
    //
            (**
             * Performs an inertia-weighted impulse based resolution of this
             * contact alone.
             *)
    Procedure applyVelocityChange(Out velocityChange: Array Of Vector3; Out
      rotationChange: Array Of Vector3);

    (**
     * Performs an inertia weighted penetration resolution of this
     * contact alone.
     *)
    Procedure applyPositionChange(Out linearChange: Array Of Vector3; Out
      angularChange: Array Of Vector3; _penetration: float);

    (**
     * Calculates the impulse needed to resolve this contact,
     * given that the contact has no friction. A pair of inertia
     * tensors - one for each contact object - is specified to
     * save calculation time: the calling function has access to
     * these anyway.
     *)
    Function calculateFrictionlessImpulse(Const inverseInertiaTensor: Array Of Matrix3): Vector3;

    (**
     * Calculates the impulse needed to resolve this contact,
     * given that the contact has a non-zero coefficient of
     * friction. A pair of inertia tensors - one for each contact
     * object - is specified to save calculation time: the calling
     * function has access to these anyway.
     *)
    Function calculateFrictionImpulse(Const inverseInertiaTensor: Array Of Matrix3): Vector3;
  End;

  pContact = ^Contact;

  (**
   * The contact resolution routine. One resolver instance
   * can be shared for the whole simulation, as long as you need
   * roughly the same parameters each time (which is normal).
   *
   * @section algorithm Resolution Algorithm
   *
   * The resolver uses an iterative satisfaction algorithm; it loops
   * through each contact and tries to resolve it. Each contact is
   * resolved locally, which may in turn put other contacts in a worse
   * position. The algorithm then revisits other contacts and repeats
   * the process up to a specified iteration limit. It can be proved
   * that given enough iterations, the simulation will get to the
   * correct result. As with all approaches, numerical stability can
   * cause problems that make a correct resolution impossible.
   *
   * @subsection strengths Strengths
   *
   * This algorithm is very fast, much faster than other physics
   * approaches. Even using many more iterations than there are
   * contacts, it will be faster than global approaches.
   *
   * Many global algorithms are unstable under high friction, this
   * approach is very robust indeed for high friction and low
   * restitution values.
   *
   * The algorithm produces visually believable behaviour. Tradeoffs
   * have been made to err on the side of visual realism rather than
   * computational expense or numerical accuracy.
   *
   * @subsection weaknesses Weaknesses
   *
   * The algorithm does not cope well with situations with many
   * inter-related contacts: stacked boxes, for example. In this
   * case the simulation may appear to jiggle slightly, which often
   * dislodges a box from the stack, allowing it to collapse.
   *
   * Another issue with the resolution mechanism is that resolving
   * one contact may make another contact move sideways against
   * friction, because each contact is handled independently, this
   * friction is not taken into account. If one object is pushing
   * against another, the pushed object may move across its support
   * without friction, even though friction is set between those bodies.
   *
   * In general this resolver is not suitable for stacks of bodies,
   * but is perfect for handling impact, explosive, and flat resting
   * situations.
   *)

  { ContactResolver }

  ContactResolver = Class
  protected
    (**
     * Holds the number of iterations to perform when resolving
     * velocity.
     *)
    velocityIterations: unsigned;

    (**
     * Holds the number of iterations to perform when resolving
     * position.
     *)
    positionIterations: unsigned;

    (**
     * To avoid instability velocities smaller
     * than this value are considered to be zero. Too small and the
     * simulation may be unstable, too large and the bodies may
     * interpenetrate visually. A good starting point is the default
     * of 0.01.
     *)
    velocityEpsilon: Float;

    (**
     * To avoid instability penetrations
     * smaller than this value are considered to be not interpenetrating.
     * Too small and the simulation may be unstable, too large and the
     * bodies may interpenetrate visually. A good starting point is
     * the default of0.01.
     *)
    positionEpsilon: Float;

  public
    (**
     * Stores the number of velocity iterations used in the
     * last call to resolve contacts.
     *)
    velocityIterationsUsed: unsigned;

    (**
     * Stores the number of position iterations used in the
     * last call to resolve contacts.
     *)
    positionIterationsUsed: unsigned;

  private
    //        /**
    //         * Keeps track of whether the internal settings are valid.
    //         */
    //        bool validSettings;
    //
  public
    (**
     * Creates a new contact resolver with the given number of iterations
     * per resolution call, and optional epsilon values.
     *)
    Constructor Create(iterations: unsigned;
      avelocityEpsilon: float = 0.01;
      apositionEpsilon: float = 0.01);

    //        /**
    //         * Creates a new contact resolver with the given number of iterations
    //         * for each kind of resolution, and optional epsilon values.
    //         */
    //        ContactResolver(unsigned velocityIterations,
    //            unsigned positionIterations,
    //            real velocityEpsilon=(real)0.01,
    //            real positionEpsilon=(real)0.01);
    //
            (**
             * Returns true if the resolver has valid settings and is ready to go.
             *)
    Function isValid(): Boolean;

    (**
     * Sets the number of iterations for each resolution stage.
     *)
    Procedure setIterations(avelocityIterations: unsigned;
      apositionIterations: unsigned);

    //        /**
    //         * Sets the number of iterations for both resolution stages.
    //         */
    //        void setIterations(unsigned iterations);
    //
            (**
             * Sets the tolerance value for both velocity and position.
             *)
    Procedure setEpsilon(avelocityEpsilon: float;
      apositionEpsilon: Float);

    (**
     * Resolves a set of contacts for both penetration and velocity.
     *
     * Contacts that cannot interact with
     * each other should be passed to separate calls to resolveContacts,
     * as the resolution algorithm takes much longer for lots of
     * contacts than it does for the same number of contacts in small
     * sets.
     *
     * @param contactArray Pointer to an array of contact objects.
     *
     * @param numContacts The number of contacts in the array to resolve.
     *
     * @param numIterations The number of iterations through the
     * resolution algorithm. This should be at least the number of
     * contacts (otherwise some constraints will not be resolved -
     * although sometimes this is not noticable). If the iterations are
     * not needed they will not be used, so adding more iterations may
     * not make any difference. In some cases you would need millions
     * of iterations. Think about the number of iterations as a bound:
     * if you specify a large number, sometimes the algorithm WILL use
     * it, and you may drop lots of frames.
     *
     * @param duration The duration of the previous integration step.
     * This is used to compensate for forces applied.
     *)
    Procedure resolveContacts(contactArray: PContact;
      numContacts: unsigned;
      duration: float);

  protected
    (**
     * Sets up contacts ready for processing. This makes sure their
     * internal data is configured correctly and the correct set of bodies
     * is made alive.
     *)
    Procedure prepareContacts(contactArray: PContact; numContacts: unsigned;
      duration: Float);

    (**
     * Resolves the velocity issues with the given array of constraints,
     * using the given number of iterations.
     *)
    Procedure adjustVelocities(c: PContact;
      numContacts: unsigned;
      duration: Float);

    (**
     * Resolves the positional issues with the given array of constraints,
     * using the given number of iterations.
     *)
    Procedure adjustPositions(c: PContact; numContacts: unsigned; duration: float);
  End;

  (**
   * This is the basic polymorphic interface for contact generators
   * applying to rigid bodies.
   *)
  ContactGenerator = Class
  public
    (**
     * Fills the given contact structure with the generated
     * contact. The contact pointer should point to the first
     * available contact in a contact array, where limit is the
     * maximum number of contacts in the array that can be written
     * to. The method returns the number of contacts that have
     * been written.
     *)
    Function addContact(Const contact: PContact; limit: unsigned): unsigned; virtual; abstract;
  End;

Implementation

{ ContactResolver }

Constructor ContactResolver.Create(iterations: unsigned;
  avelocityEpsilon: float; apositionEpsilon: float);
Begin
  Inherited Create;
  setIterations(iterations, iterations);
  setEpsilon(avelocityEpsilon, apositionEpsilon);
End;

Function ContactResolver.isValid: Boolean;
Begin
  result :=
    (velocityIterations > 0) And
    (positionIterations > 0) And
    (positionEpsilon >= 0.0) And
    (positionEpsilon >= 0.0);
End;

Procedure ContactResolver.setIterations(avelocityIterations: unsigned;
  apositionIterations: unsigned);
Begin
  velocityIterations := avelocityIterations;
  positionIterations := apositionIterations;
End;

Procedure ContactResolver.setEpsilon(avelocityEpsilon: float;
  apositionEpsilon: Float);
Begin
  velocityEpsilon := avelocityEpsilon;
  positionEpsilon := apositionEpsilon;
End;

Procedure ContactResolver.resolveContacts(contactArray: PContact;
  numContacts: unsigned; duration: float);
Begin
  // Make sure we have something to do.
  If (numContacts = 0) Then exit;
  If (Not isValid()) Then exit;

  // Prepare the contacts for processing
  prepareContacts(contactArray, numContacts, duration);

  // Resolve the interpenetration problems with the contacts.
  adjustPositions(contactArray, numContacts, duration);

  // Resolve the velocity problems with the contacts.
  adjustVelocities(contactArray, numContacts, duration);
End;

Procedure ContactResolver.prepareContacts(contactArray: PContact;
  numContacts: unsigned; duration: Float);
Var
  i: integer;
  contact: pContact;
Begin
  // Generate contact velocity and axis information.
  contact := contactArray;
  For i := 0 To numContacts - 1 Do Begin
    // Calculate the internal contact data (inertia, basis, etc).
    contact^.calculateInternals(duration);
    inc(contact);
  End;
End;

Procedure ContactResolver.adjustVelocities(c: PContact;
  numContacts: unsigned; duration: Float);
Var
  velocityChange, rotationChange: Array[0..1] Of Vector3;
  deltaVel: Vector3;
  _max: float;
  d, b, i, index: unsigned;
Begin
  // iteratively handle impacts in order of severity.
  velocityIterationsUsed := 0;
  While (velocityIterationsUsed < velocityIterations) Do Begin

    // Find contact with maximum magnitude of probable velocity change.
    _max := velocityEpsilon;
    index := numContacts;

    For i := 0 To numContacts - 1 Do Begin
      If (c[i].desiredDeltaVelocity > _max) Then Begin
        _max := c[i].desiredDeltaVelocity;
        index := i;
      End;
    End;
    If (index = numContacts) Then break;

    // Match the awake state at the contact
    c[index].matchAwakeState();

    // Do the resolution on the contact that came out top.
    c[index].applyVelocityChange(velocityChange, rotationChange);

    // With the change in velocity of the two bodies, the update of
    // contact velocities means that some of the relative closing
    // velocities need recomputing.

    For i := 0 To numContacts - 1 Do Begin
      // Check each body in the contact
      For b := 0 To 1 Do Begin
        If assigned(c[i].body[b]) Then Begin
          // Check for a match with each body in the newly
          // resolved contact

          For d := 0 To 1 Do Begin

            If (c[i].body[b] = c[index].body[d]) Then Begin

              deltaVel := velocityChange[d] +
                rotationChange[d].vectorProduct(
                c[i].relativeContactPosition[b]);

              // The sign of the change is negative if we're dealing
              // with the second body in a contact.
              c[i].contactVelocity := c[i].contactVelocity +
                c[i].contactToWorld.transformTranspose(deltaVel)
                * ifthen(b <> 0, -1, 1);
              c[i].calculateDesiredDeltaVelocity(duration);
            End;
          End;
        End;
      End;
    End;
    velocityIterationsUsed := velocityIterationsUsed + 1;
  End;
End;

Procedure ContactResolver.adjustPositions(c: PContact;
  numContacts: unsigned; duration: float);
Var
  index, i, b, d: unsigned;
  linearChange: Array[0..1] Of Vector3;
  angularChange: Array[0..1] Of Vector3;
  _max: float;
  deltaPosition: Vector3;
Begin
  // iteratively resolve interpenetrations in order of severity.
  positionIterationsUsed := 0;
  While (positionIterationsUsed < positionIterations) Do Begin

    // Find biggest penetration
    _max := positionEpsilon;
    index := numContacts;
    For i := 0 To numContacts - 1 Do Begin
      If (c[i].penetration > _max) Then Begin
        _max := c[i].penetration;
        index := i;
      End;
    End;
    If (index = numContacts) Then break;

    // Match the awake state at the contact
    c[index].matchAwakeState();

    // Resolve the penetration.
    c[index].applyPositionChange(
      linearChange,
      angularChange,
      _max);

    // Again this action may have changed the penetration of other
    // bodies, so we update contacts.
    For i := 0 To numContacts - 1 Do Begin
      // Check each body in the contact
      For b := 0 To 1 Do Begin
        If assigned(c[i].body[b]) Then Begin
          // Check for a match with each body in the newly
          // resolved contact
          For d := 0 To 1 Do Begin
            If (c[i].body[b] = c[index].body[d]) Then Begin
              deltaPosition := linearChange[d] +
                angularChange[d].vectorProduct(
                c[i].relativeContactPosition[b]);

              // The sign of the change is positive if we're
              // dealing with the second body in a contact
              // and negative otherwise (because we're
              // subtracting the resolution)..
              c[i].penetration := c[i].penetration +
                deltaPosition.scalarProduct(c[i].contactNormal)
                * ifthen(b <> 0, 1, -1);
            End;
          End;
        End;
      End;
    End;
    positionIterationsUsed := positionIterationsUsed + 1;
  End;
End;

Procedure Contact.setBodyData(one, two: PRigidBody; afriction,
  arestitution: Float);
Begin
  body[0] := one;
  body[1] := two;
  friction := afriction;
  restitution := arestitution;
End;

Procedure Contact.calculateInternals(duration: Float);
Begin
  // Check if the first object is NULL, and swap if it is.
  If (body[0] = Nil) Then swapBodies();
  assert(assigned(body[0]));

  // Calculate an set of axis at the contact point.
  calculateContactBasis();

  // Store the relative position of the contact relative to each body
  relativeContactPosition[0] := contactPoint - body[0]^.getPosition();
  If assigned(body[1]) Then Begin
    relativeContactPosition[1] := contactPoint - body[1]^.getPosition();
  End;

  // Find the relative velocity of the bodies at the contact point.
  contactVelocity := calculateLocalVelocity(0, duration);
  If assigned(body[1]) Then Begin
    contactVelocity := contactVelocity - calculateLocalVelocity(1, duration);
  End;

  // Calculate the desired change in velocity for resolution
  calculateDesiredDeltaVelocity(duration);
End;

Procedure Contact.swapBodies;
Var
  temp: PRigidBody;
Begin
  contactNormal := contactNormal * (-1);

  temp := body[0];
  body[0] := body[1];
  body[1] := temp;
End;

Procedure Contact.matchAwakeState;
Var
  body0awake, body1awake: Boolean;
Begin
  // Collisions with the world never cause a body to wake up.
  If (body[1] = Nil) Then exit;

  body0awake := body[0]^.getAwake();
  body1awake := body[1]^.getAwake();

  // Wake up only the sleeping one
  If (body0awake Xor body1awake) Then Begin
    If (body0awake) Then
      body[1]^.setAwake()
    Else
      body[0]^.setAwake();
  End;
End;

Procedure Contact.calculateDesiredDeltaVelocity(duration: float);
Const
  velocityLimit = 0.25;
Var
  thisRestitution, velocityFromAcc: float;

Begin
  // Calculate the acceleration induced velocity accumulated this frame
  velocityFromAcc := 0;

  If (body[0]^.getAwake()) Then Begin
    velocityFromAcc := velocityFromAcc +
      body[0]^.getLastFrameAcceleration() * duration * contactNormal;
  End;

  If assigned(body[1]) And (body[1]^.getAwake()) Then Begin
    velocityFromAcc := velocityFromAcc -
      body[1]^.getLastFrameAcceleration() * duration * contactNormal;
  End;

  // If the velocity is very slow, limit the restitution
  thisRestitution := restitution;
  If (real_abs(contactVelocity.x) < velocityLimit) Then Begin
    thisRestitution := 0.0;
  End;

  // Combine the bounce velocity with the removed
  // acceleration velocity.
  desiredDeltaVelocity :=
    -contactVelocity.x
    - thisRestitution * (contactVelocity.x - velocityFromAcc);
End;

Function Contact.calculateLocalVelocity(bodyIndex: unsigned; duration: Float
  ): Vector3;
Var
  thisBody: PRigidBody;
  accVelocity, _contactVelocity, velocity: Vector3;
Begin
  thisBody := body[bodyIndex];

  // Work out the velocity of the contact point.
  velocity :=
    thisBody^.getRotation() Mod relativeContactPosition[bodyIndex];
  velocity := velocity + thisBody^.getVelocity();

  // Turn the velocity into contact-coordinates.
  _contactVelocity := contactToWorld.transformTranspose(velocity);

  // Calculate the ammount of velocity that is due to forces without
  // reactions.
  accVelocity := thisBody^.getLastFrameAcceleration() * duration;

  // Calculate the velocity in contact-coordinates.
  accVelocity := contactToWorld.transformTranspose(accVelocity);

  // We ignore any component of acceleration in the contact normal
  // direction, we are only interested in planar acceleration
  accVelocity.x := 0;

  // Add the planar velocities - if there's enough friction they will
  // be removed during velocity resolution
  _contactVelocity := contactVelocity + accVelocity;

  // And return it
  result := _contactVelocity;
End;

(*
 * Constructs an arbitrary orthonormal basis for the contact.  This is
 * stored as a 3x3 matrix, where each vector is a column (in other
 * words the matrix transforms contact space into world space). The x
 * direction is generated from the contact normal, and the y and z
 * directionss are set so they are at right angles to it.
 *)

Procedure Contact.calculateContactBasis;
Var
  contactTangent: Array[0..1] Of Vector3;
  s: Float;
Begin
  // Check whether the Z-axis is nearer to the X or Y axis
  If (real_abs(contactNormal.x) > real_abs(contactNormal.y)) Then Begin

    // Scaling factor to ensure the results are normalised
    s := 1.0 / real_sqrt(contactNormal.z * contactNormal.z +
      contactNormal.x * contactNormal.x);

    // The new X-axis is at right angles to the world Y-axis
    contactTangent[0].x := contactNormal.z * s;
    contactTangent[0].y := 0;
    contactTangent[0].z := -contactNormal.x * s;

    // The new Y-axis is at right angles to the new X- and Z- axes
    contactTangent[1].x := contactNormal.y * contactTangent[0].z;
    contactTangent[1].y := contactNormal.z * contactTangent[0].x -
      contactNormal.x * contactTangent[0].z;
    contactTangent[1].z := -contactNormal.y * contactTangent[0].x;
  End
  Else Begin
    // Scaling factor to ensure the results are normalised
    s := 1.0 / real_sqrt(contactNormal.z * contactNormal.z +
      contactNormal.y * contactNormal.y);

    // The new X-axis is at right angles to the world X-axis
    contactTangent[0].x := 0;
    contactTangent[0].y := -contactNormal.z * s;
    contactTangent[0].z := contactNormal.y * s;

    // The new Y-axis is at right angles to the new X- and Z- axes
    contactTangent[1].x := contactNormal.y * contactTangent[0].z -
      contactNormal.z * contactTangent[0].y;
    contactTangent[1].y := -contactNormal.x * contactTangent[0].z;
    contactTangent[1].z := contactNormal.x * contactTangent[0].y;
  End;

  // Make a matrix from the three vectors.
  contactToWorld.setComponents(
    contactNormal,
    contactTangent[0],
    contactTangent[1]);
End;

Procedure Contact.applyVelocityChange(Out velocityChange: Array Of Vector3;
  Out rotationChange: Array Of Vector3);
Var
  inverseInertiaTensor: Array[0..1] Of Matrix3;
  impulsiveTorque, impulse, impulseContact: Vector3;
Begin
  // Get hold of the inverse mass and inverse inertia tensor, both in
  // world coordinates.

  body[0]^.getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
  If assigned(body[1]) Then
    body[1]^.getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

  // We will calculate the impulse for each contact axis
  If (friction = 0.0) Then Begin
    // Use the short format for frictionless contacts
    impulseContact := calculateFrictionlessImpulse(inverseInertiaTensor);
  End
  Else Begin
    // Otherwise we may have impulses that aren't in the direction of the
    // contact, so we need the more complex version.
    impulseContact := calculateFrictionImpulse(inverseInertiaTensor);
  End;

  // Convert impulse to world coordinates
  impulse := contactToWorld.transform(impulseContact);

  // Split in the impulse into linear and rotational components
  impulsiveTorque := relativeContactPosition[0] Mod impulse;
  rotationChange[0] := inverseInertiaTensor[0].transform(impulsiveTorque);
  velocityChange[0] := V3(0, 0, 0); // .clear ohne Warnung ;)
  velocityChange[0].addScaledVector(impulse, body[0]^.getInverseMass());

  // Apply the changes
  body[0]^.addVelocity(velocityChange[0]);
  body[0]^.addRotation(rotationChange[0]);

  If assigned(body[1]) Then Begin

    // Work out body one's linear and angular changes
    impulsiveTorque := impulse Mod relativeContactPosition[1];
    rotationChange[1] := inverseInertiaTensor[1].transform(impulsiveTorque);
    velocityChange[1].clear();
    velocityChange[1].addScaledVector(impulse, -body[1]^.getInverseMass());

    // And apply them.
    body[1]^.addVelocity(velocityChange[1]);
    body[1]^.addRotation(rotationChange[1]);
  End;
End;

Procedure Contact.applyPositionChange(Out linearChange: Array Of Vector3; Out
  angularChange: Array Of Vector3; _penetration: float);
Const
  angularLimit = 0.2;
Var
  angularMove: Array[0..1] Of float;
  linearMove: Array[0..1] Of float;

  totalMove, maxMagnitude, _sign, totalInertia: float;
  linearInertia: Array[0..1] Of float;
  angularInertia: Array[0..1] Of float;
  i: unsigned;
  inverseInertiaTensor: Matrix3;
  pos, targetAngularDirection, projection, angularInertiaWorld: Vector3;
  q: Quaternion;
Begin

  totalInertia := 0;

  // We need to work out the inertia of each object in the direction
  // of the contact normal, due to angular inertia only.
  For i := 0 To 1 Do Begin
    If assigned(body[i]) Then Begin
      body[i]^.getInverseInertiaTensorWorld(inverseInertiaTensor);

      // Use the same procedure as for calculating frictionless
      // velocity change to work out the angular inertia.
      angularInertiaWorld :=
        relativeContactPosition[i] Mod contactNormal;
      angularInertiaWorld :=
        inverseInertiaTensor.transform(angularInertiaWorld);
      angularInertiaWorld :=
        angularInertiaWorld Mod relativeContactPosition[i];
      angularInertia[i] :=
        angularInertiaWorld * contactNormal;

      // The linear component is simply the inverse mass
      linearInertia[i] := body[i]^.getInverseMass();

      // Keep track of the total inertia from all components
      totalInertia := totalInertia + linearInertia[i] + angularInertia[i];

      // We break the loop here so that the totalInertia value is
      // completely calculated (by both iterations) before
      // continuing.
    End;
  End;

  // Loop through again calculating and applying the changes
  For i := 0 To 1 Do Begin
    If assigned(body[i]) Then Begin
      // The linear and angular movements required are in proportion to
      // the two inverse inertias.
      _sign := IfThen((i = 0), 1, -1);
      angularMove[i] :=
        _sign * _penetration * (angularInertia[i] / totalInertia);
      linearMove[i] :=
        _sign * _penetration * (linearInertia[i] / totalInertia);

      // To avoid angular projections that are too great (when mass is large
      // but inertia tensor is small) limit the angular move.
      projection := relativeContactPosition[i];
      projection.addScaledVector(
        contactNormal,
        -relativeContactPosition[i].scalarProduct(contactNormal)
        );

      // Use the small angle approximation for the sine of the angle (i.e.
      // the magnitude would be sine(angularLimit) * projection.magnitude
      // but we approximate sine(angularLimit) to angularLimit).
      maxMagnitude := angularLimit * projection.magnitude();

      If (angularMove[i] < -maxMagnitude) Then Begin
        totalMove := angularMove[i] + linearMove[i];
        angularMove[i] := -maxMagnitude;
        linearMove[i] := totalMove - angularMove[i];
      End
      Else If (angularMove[i] > maxMagnitude) Then Begin
        totalMove := angularMove[i] + linearMove[i];
        angularMove[i] := maxMagnitude;
        linearMove[i] := totalMove - angularMove[i];
      End;

      // We have the linear amount of movement required by turning
      // the rigid body (in angularMove[i]). We now need to
      // calculate the desired rotation to achieve that.
      If (angularMove[i] = 0) Then Begin

        // Easy case - no angular movement means no rotation.
        angularChange[i] := v3(0, 0, 0); // .clear(); ohne Warnung
      End
      Else Begin
        // Work out the direction we'd like to rotate in.
        targetAngularDirection :=
          relativeContactPosition[i].vectorProduct(contactNormal);


        body[i]^.getInverseInertiaTensorWorld(inverseInertiaTensor);

        // Work out the direction we'd need to rotate to achieve that
        angularChange[i] :=
          inverseInertiaTensor.transform(targetAngularDirection) *
          (angularMove[i] / angularInertia[i]);
      End;

      // Velocity change is easier - it is just the linear movement
      // along the contact normal.
      linearChange[i] := contactNormal * linearMove[i];

      // Now we can start to apply the values we've calculated.
      // Apply the linear movement

      body[i]^.getPosition(&pos);
      pos.addScaledVector(contactNormal, linearMove[i]);
      body[i]^.setPosition(pos);

      // And the change in orientation

      body[i]^.getOrientation(q);
      q.addScaledVector(angularChange[i], 1.0);
      body[i]^.setOrientation(q);

      // We need to calculate the derived data for any body that is
      // asleep, so that the changes are reflected in the object's
      // data. Otherwise the resolution will not change the position
      // of the object, and the next collision detection round will
      // have the same penetration.
      If (Not body[i]^.getAwake()) Then body[i]^.calculateDerivedData();
    End;
  End;
End;

Function Contact.calculateFrictionlessImpulse(
  Const inverseInertiaTensor: Array Of Matrix3): Vector3;
Begin
  Raise exception.create('Hier weiter');
  //Vector3 impulseContact;
  //
  // // Build a vector that shows the change in velocity in
  // // world space for a unit impulse in the direction of the contact
  // // normal.
  // Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
  // deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
  // deltaVelWorld = deltaVelWorld % relativeContactPosition[0];
  //
  // // Work out the change in velocity in contact coordiantes.
  // real deltaVelocity = deltaVelWorld * contactNormal;
  //
  // // Add the linear component of velocity change
  // deltaVelocity += body[0]->getInverseMass();
  //
  // // Check if we need to the second body's data
  // if (body[1])
  // {
  //     // Go through the same transformation sequence again
  //     Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
  //     deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
  //     deltaVelWorld = deltaVelWorld % relativeContactPosition[1];
  //
  //     // Add the change in velocity due to rotation
  //     deltaVelocity += deltaVelWorld * contactNormal;
  //
  //     // Add the change in velocity due to linear motion
  //     deltaVelocity += body[1]->getInverseMass();
  // }
  //
  // // Calculate the required size of the impulse
  // impulseContact.x = desiredDeltaVelocity / deltaVelocity;
  // impulseContact.y = 0;
  // impulseContact.z = 0;
  // return impulseContact;
End;

Function Contact.calculateFrictionImpulse(
  Const inverseInertiaTensor: Array Of Matrix3): Vector3;
Begin
  Raise exception.create('Hier weiter');
  //Vector3 impulseContact;
  // real inverseMass = body[0]->getInverseMass();
  //
  // // The equivalent of a cross product in matrices is multiplication
  // // by a skew symmetric matrix - we build the matrix for converting
  // // between linear and angular quantities.
  // Matrix3 impulseToTorque;
  // impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);
  //
  // // Build the matrix to convert contact impulse to change in velocity
  // // in world coordinates.
  // Matrix3 deltaVelWorld = impulseToTorque;
  // deltaVelWorld *= inverseInertiaTensor[0];
  // deltaVelWorld *= impulseToTorque;
  // deltaVelWorld *= -1;
  //
  // // Check if we need to add body two's data
  // if (body[1])
  // {
  //     // Set the cross product matrix
  //     impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);
  //
  //     // Calculate the velocity change matrix
  //     Matrix3 deltaVelWorld2 = impulseToTorque;
  //     deltaVelWorld2 *= inverseInertiaTensor[1];
  //     deltaVelWorld2 *= impulseToTorque;
  //     deltaVelWorld2 *= -1;
  //
  //     // Add to the total delta velocity.
  //     deltaVelWorld += deltaVelWorld2;
  //
  //     // Add to the inverse mass
  //     inverseMass += body[1]->getInverseMass();
  // }
  //
  // // Do a change of basis to convert into contact coordinates.
  // Matrix3 deltaVelocity = contactToWorld.transpose();
  // deltaVelocity *= deltaVelWorld;
  // deltaVelocity *= contactToWorld;
  //
  // // Add in the linear velocity change
  // deltaVelocity.data[0] += inverseMass;
  // deltaVelocity.data[4] += inverseMass;
  // deltaVelocity.data[8] += inverseMass;
  //
  // // Invert to get the impulse needed per unit velocity
  // Matrix3 impulseMatrix = deltaVelocity.inverse();
  //
  // // Find the target velocities to kill
  // Vector3 velKill(desiredDeltaVelocity,
  //     -contactVelocity.y,
  //     -contactVelocity.z);
  //
  // // Find the impulse to kill target velocities
  // impulseContact = impulseMatrix.transform(velKill);
  //
  // // Check for exceeding friction
  // real planarImpulse = real_sqrt(
  //     impulseContact.y*impulseContact.y +
  //     impulseContact.z*impulseContact.z
  //     );
  // if (planarImpulse > impulseContact.x * friction)
  // {
  //     // We need to use dynamic friction
  //     impulseContact.y /= planarImpulse;
  //     impulseContact.z /= planarImpulse;
  //
  //     impulseContact.x = deltaVelocity.data[0] +
  //         deltaVelocity.data[1]*friction*impulseContact.y +
  //         deltaVelocity.data[2]*friction*impulseContact.z;
  //     impulseContact.x = desiredDeltaVelocity / impulseContact.x;
  //     impulseContact.y *= friction * impulseContact.x;
  //     impulseContact.z *= friction * impulseContact.x;
  // }
  // return impulseContact;
End;

End.

