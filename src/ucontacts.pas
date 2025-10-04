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

    //        /**
    //         * Updates the awake state of rigid bodies that are taking
    //         * place in the given contact. A body will be made awake if it
    //         * is in contact with a body that is awake.
    //         */
    //        void matchAwakeState();

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
    //        /**
    //         * Performs an inertia-weighted impulse based resolution of this
    //         * contact alone.
    //         */
    //        void applyVelocityChange(Vector3 velocityChange[2],
    //                                 Vector3 rotationChange[2]);
    //
    //        /**
    //         * Performs an inertia weighted penetration resolution of this
    //         * contact alone.
    //         */
    //        void applyPositionChange(Vector3 linearChange[2],
    //                                 Vector3 angularChange[2],
    //                                 real penetration);
    //
    //        /**
    //         * Calculates the impulse needed to resolve this contact,
    //         * given that the contact has no friction. A pair of inertia
    //         * tensors - one for each contact object - is specified to
    //         * save calculation time: the calling function has access to
    //         * these anyway.
    //         */
    //        Vector3 calculateFrictionlessImpulse(Matrix3 *inverseInertiaTensor);
    //
    //        /**
    //         * Calculates the impulse needed to resolve this contact,
    //         * given that the contact has a non-zero coefficient of
    //         * friction. A pair of inertia tensors - one for each contact
    //         * object - is specified to save calculation time: the calling
    //         * function has access to these anyway.
    //         */
    //        Vector3 calculateFrictionImpulse(Matrix3 *inverseInertiaTensor);
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
    //        /**
    //         * Stores the number of velocity iterations used in the
    //         * last call to resolve contacts.
    //         */
    //        unsigned velocityIterationsUsed;
    //
    //        /**
    //         * Stores the number of position iterations used in the
    //         * last call to resolve contacts.
    //         */
    //        unsigned positionIterationsUsed;
    //
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
    Procedure adjustVelocities(contactArray: PContact;
      numContacts: unsigned;
      duration: Float);

    (**
     * Resolves the positional issues with the given array of constraints,
     * using the given number of iterations.
     *)
    Procedure adjustPositions(contacts: PContact;
      numContacts: unsigned;
      duration: float);
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

Procedure ContactResolver.adjustVelocities(contactArray: PContact;
  numContacts: unsigned; duration: Float);
Begin
  //    Vector3 velocityChange[2], rotationChange[2];
  //    Vector3 deltaVel;
  //
  //    // iteratively handle impacts in order of severity.
  //    velocityIterationsUsed = 0;
  //    while (velocityIterationsUsed < velocityIterations)
  //    {
  //        // Find contact with maximum magnitude of probable velocity change.
  //        real max = velocityEpsilon;
  //        unsigned index = numContacts;
  //        for (unsigned i = 0; i < numContacts; i++)
  //        {
  //            if (c[i].desiredDeltaVelocity > max)
  //            {
  //                max = c[i].desiredDeltaVelocity;
  //                index = i;
  //            }
  //        }
  //        if (index == numContacts) break;
  //
  //        // Match the awake state at the contact
  //        c[index].matchAwakeState();
  //
  //        // Do the resolution on the contact that came out top.
  //        c[index].applyVelocityChange(velocityChange, rotationChange);
  //
  //        // With the change in velocity of the two bodies, the update of
  //        // contact velocities means that some of the relative closing
  //        // velocities need recomputing.
  //        for (unsigned i = 0; i < numContacts; i++)
  //        {
  //            // Check each body in the contact
  //            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
  //            {
  //                // Check for a match with each body in the newly
  //                // resolved contact
  //                for (unsigned d = 0; d < 2; d++)
  //                {
  //                    if (c[i].body[b] == c[index].body[d])
  //                    {
  //                        deltaVel = velocityChange[d] +
  //                            rotationChange[d].vectorProduct(
  //                                c[i].relativeContactPosition[b]);
  //
  //                        // The sign of the change is negative if we're dealing
  //                        // with the second body in a contact.
  //                        c[i].contactVelocity +=
  //                            c[i].contactToWorld.transformTranspose(deltaVel)
  //                            * (b?-1:1);
  //                        c[i].calculateDesiredDeltaVelocity(duration);
  //                    }
  //                }
  //            }
  //        }
  //        velocityIterationsUsed++;
  //    }
End;

Procedure ContactResolver.adjustPositions(contacts: PContact;
  numContacts: unsigned; duration: float);
Begin
  //    unsigned i,index;
  //    Vector3 linearChange[2], angularChange[2];
  //    real max;
  //    Vector3 deltaPosition;
  //
  //    // iteratively resolve interpenetrations in order of severity.
  //    positionIterationsUsed = 0;
  //    while (positionIterationsUsed < positionIterations)
  //    {
  //        // Find biggest penetration
  //        max = positionEpsilon;
  //        index = numContacts;
  //        for (i=0; i<numContacts; i++)
  //        {
  //            if (c[i].penetration > max)
  //            {
  //                max = c[i].penetration;
  //                index = i;
  //            }
  //        }
  //        if (index == numContacts) break;
  //
  //        // Match the awake state at the contact
  //        c[index].matchAwakeState();
  //
  //        // Resolve the penetration.
  //        c[index].applyPositionChange(
  //            linearChange,
  //            angularChange,
  //            max);
  //
  //        // Again this action may have changed the penetration of other
  //        // bodies, so we update contacts.
  //        for (i = 0; i < numContacts; i++)
  //        {
  //            // Check each body in the contact
  //            for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
  //            {
  //                // Check for a match with each body in the newly
  //                // resolved contact
  //                for (unsigned d = 0; d < 2; d++)
  //                {
  //                    if (c[i].body[b] == c[index].body[d])
  //                    {
  //                        deltaPosition = linearChange[d] +
  //                            angularChange[d].vectorProduct(
  //                                c[i].relativeContactPosition[b]);
  //
  //                        // The sign of the change is positive if we're
  //                        // dealing with the second body in a contact
  //                        // and negative otherwise (because we're
  //                        // subtracting the resolution)..
  //                        c[i].penetration +=
  //                            deltaPosition.scalarProduct(c[i].contactNormal)
  //                            * (b?1:-1);
  //                    }
  //                }
  //            }
  //        }
  //        positionIterationsUsed++;
  //    }
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

Procedure Contact.calculateDesiredDeltaVelocity(duration: float);
Begin
  //hier weiter
  //    const static real velocityLimit = (real)0.25f;
  //
  //    // Calculate the acceleration induced velocity accumulated this frame
  //    real velocityFromAcc = 0;
  //
  //    if (body[0]->getAwake())
  //    {
  //	velocityFromAcc+=
  //	    body[0]->getLastFrameAcceleration() * duration * contactNormal;
  //    }
  //
  //    if (body[1] && body[1]->getAwake())
  //    {
  //        velocityFromAcc -=
  //            body[1]->getLastFrameAcceleration() * duration * contactNormal;
  //    }
  //
  //    // If the velocity is very slow, limit the restitution
  //    real thisRestitution = restitution;
  //    if (real_abs(contactVelocity.x) < velocityLimit)
  //    {
  //        thisRestitution = (real)0.0f;
  //    }
  //
  //    // Combine the bounce velocity with the removed
  //    // acceleration velocity.
  //    desiredDeltaVelocity =
  //        -contactVelocity.x
  //        -thisRestitution * (contactVelocity.x - velocityFromAcc);
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

End.

