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
Unit ubody;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils
  , ucore, uprecision;

Type

  (**
    * A rigid body is the basic simulation object in the physics
    * core.
    *
    * It has position and orientation data, along with first
    * derivatives. It can be integrated forward through time, and
    * have forces, torques and impulses (linear or angular) applied
    * to it. The rigid body manages its state and allows access
    * through a set of methods.
    *
    * A ridid body contains 64 words (the size of which is given
    * by the precision: sizeof(real)). It contains no virtual
    * functions, so should take up exactly 64 words in memory. Of
    * this total 15 words are padding, distributed among the
    * Vector3 data members.
    *)

  { RigidBody }

  RigidBody = Class

  public

    // ... Other RigidBody code as before ...


  protected
    (**
     * @name Characteristic Data and State
     *
     * This data holds the state of the rigid body. There are two
     * sets of data: characteristics and state.
     *
     * Characteristics are properties of the rigid body
     * independent of its current kinematic situation. This
     * includes mass, moment of inertia and damping
     * properties. Two identical rigid bodys will have the same
     * values for their characteristics.
     *
     * State includes all the characteristics and also includes
     * the kinematic situation of the rigid body in the current
     * simulation. By setting the whole state data, a rigid body's
     * exact game state can be replicated. Note that state does
     * not include any forces applied to the body. Two identical
     * rigid bodies in the same simulation will not share the same
     * state values.
     *
     * The state values make up the smallest set of independent
     * data for the rigid body. Other state data is calculated
     * from their current values. When state data is changed the
     * dependent values need to be updated: this can be achieved
     * either by integrating the simulation, or by calling the
     * calculateInternals function. This two stage process is used
     * because recalculating internals can be a costly process:
     * all state changes should be carried out at the same time,
     * allowing for a single call.
     *
     * @see calculateInternals
     *)
    (*@{*)
    (**
     * Holds the inverse of the mass of the rigid body. It
     * is more useful to hold the inverse mass because
     * integration is simpler, and because in real time
     * simulation it is more useful to have bodies with
     * infinite mass (immovable) than zero mass
     * (completely unstable in numerical simulation).
     *)
    inverseMass: float;

    (**
     * Holds the inverse of the body's inertia tensor. The
     * inertia tensor provided must not be degenerate
     * (that would mean the body had zero inertia for
     * spinning along one axis). As long as the tensor is
     * finite, it will be invertible. The inverse tensor
     * is used for similar reasons to the use of inverse
     * mass.
     *
     * The inertia tensor, unlike the other variables that
     * define a rigid body, is given in body space.
     *
     * @see inverseMass
     *)
    inverseInertiaTensor: Matrix3;

    (**
     * Holds the amount of damping applied to linear
     * motion.  Damping is required to remove energy added
     * through numerical instability in the integrator.
     *)
    linearDamping: float;

    (**
     * Holds the amount of damping applied to angular
     * motion.  Damping is required to remove energy added
     * through numerical instability in the integrator.
     *)
    angularDamping: float;

    (**
     * Holds the linear position of the rigid body in
     * world space.
     *)
    position: Vector3;

    (**
     * Holds the angular orientation of the rigid body in
     * world space.
     *)
    orientation: Quaternion;

    (**
     * Holds the linear velocity of the rigid body in
     * world space.
     *)
    velocity: Vector3;

    (**
     * Holds the angular velocity, or rotation, or the
     * rigid body in world space.
     *)
    rotation: Vector3;

    (*@}*)

    (**
     * @name Derived Data
     *
     * These data members hold information that is derived from
     * the other data in the class.
     *)
    (*@{*)

    (**
     * Holds the inverse inertia tensor of the body in world
     * space. The inverse inertia tensor member is specified in
     * the body's local space.
     *
     * @see inverseInertiaTensor
     *)
    inverseInertiaTensorWorld: Matrix3;

    (**
     * Holds the amount of motion of the body. This is a recency
     * weighted mean that can be used to put a body to sleap.
     *)
    motion: float;

    (**
     * A body can be put to sleep to avoid it being updated
     * by the integration functions or affected by collisions
     * with the world.
     *)
    isAwake: boolean;

    (**
     * Some bodies may never be allowed to fall asleep.
     * User controlled bodies, for example, should be
     * always awake.
     *)
    canSleep: Boolean;

    (**
     * Holds a transform matrix for converting body space into
     * world space and vice versa. This can be achieved by calling
     * the getPointIn*Space functions.
     *
     * @see getPointInLocalSpace
     * @see getPointInWorldSpace
     * @see getTransform
     *)
    transformMatrix: Matrix4;

    (*@}*)


    (**
     * @name Force and Torque Accumulators
     *
     * These data members store the current force, torque and
     * acceleration of the rigid body. Forces can be added to the
     * rigid body in any order, and the class decomposes them into
     * their constituents, accumulating them for the next
     * simulation step. At the simulation step, the accelerations
     * are calculated and stored to be applied to the rigid body.
     *)
    (*@{*)

    (**
     * Holds the accumulated force to be applied at the next
     * integration step.
     *)
    forceAccum: Vector3;

    (**
     * Holds the accumulated torque to be applied at the next
     * integration step.
     *)
    torqueAccum: Vector3;

    (**
      * Holds the acceleration of the rigid body.  This value
      * can be used to set acceleration due to gravity (its primary
      * use), or any other constant acceleration.
      *)
    acceleration: Vector3;

    (**
     * Holds the linear acceleration of the rigid body, for the
     * previous frame.
     *)
    lastFrameAcceleration: Vector3;

    (*@}*)

  public
    (**
     * @name Constructor and Destructor
     *
     * There are no data members in the rigid body class that are
     * created on the heap. So all data storage is handled
     * automatically.
     *)
    (*@{*)
    Constructor Create(); virtual;
    Destructor Destroy(); override;
    (*@}*)


    (**
     * @name Integration and Simulation Functions
     *
     * These functions are used to simulate the rigid body's
     * motion over time. A normal application sets up one or more
     * rigid bodies, applies permanent forces (i.e. gravity), then
     * adds transient forces each frame, and integrates, prior to
     * rendering.
     *
     * Currently the only integration function provided is the
     * first order Newton Euler method.
     *)
    (*@{*)

    (**
     * Calculates internal data from state data. This should be called
     * after the body's state is altered directly (it is called
     * automatically during integration). If you change the body's state
     * and then intend to integrate before querying any data (such as
     * the transform matrix), then you can ommit this step.
     *)
    Procedure calculateDerivedData();

    (**
     * Integrates the rigid body forward in time by the given amount.
     * This function uses a Newton-Euler integration method, which is a
     * linear approximation to the correct integral. For this reason it
     * may be inaccurate in some cases.
     *)
    Procedure integrate(duration: float);

    (*@}*)


    (**
     * @name Accessor Functions for the Rigid Body's State
     *
     * These functions provide access to the rigid body's
     * characteristics or state. These data can be accessed
     * individually, or en masse as an array of values
     * (e.g. getCharacteristics, getState). When setting new data,
     * make sure the calculateInternals function, or an
     * integration routine, is called before trying to get data
     * from the body, since the class contains a number of
     * dependent values that will need recalculating.
     *)
    (*@{*)

    (**
     * Sets the mass of the rigid body.
     *
     * @param mass The new mass of the body. This may not be zero.
     * Small masses can produce unstable rigid bodies under
     * simulation.
     *
     * @warning This invalidates internal data for the rigid body.
     * Either an integration function, or the calculateInternals
     * function should be called before trying to get any settings
     * from the rigid body.
     *)
    Procedure setMass(Const mass: float);

    //        /**
    //         * Gets the mass of the rigid body.
    //         *
    //         * @return The current mass of the rigid body.
    //         */
    //        real getMass() const;
    //
    //        /**
    //         * Sets the inverse mass of the rigid body.
    //         *
    //         * @param inverseMass The new inverse mass of the body. This
    //         * may be zero, for a body with infinite mass
    //         * (i.e. unmovable).
    //         *
    //         * @warning This invalidates internal data for the rigid body.
    //         * Either an integration function, or the calculateInternals
    //         * function should be called before trying to get any settings
    //         * from the rigid body.
    //         */
    //        void setInverseMass(const real inverseMass);

            (**
             * Gets the inverse mass of the rigid body.
             *
             * @return The current inverse mass of the rigid body.
             *)
    Function getInverseMass(): float;

    //        /**
    //         * Returns true if the mass of the body is not-infinite.
    //         */
    //        bool hasFiniteMass() const;
    //
            (**
             * Sets the intertia tensor for the rigid body.
             *
             * @param inertiaTensor The inertia tensor for the rigid
             * body. This must be a full rank matrix and must be
             * invertible.
             *
             * @warning This invalidates internal data for the rigid body.
             * Either an integration function, or the calculateInternals
             * function should be called before trying to get any settings
             * from the rigid body.
             *)
    Procedure setInertiaTensor(Const inertiaTensor: Matrix3);

    //        /**
    //         * Copies the current inertia tensor of the rigid body into
    //         * the given matrix.
    //         *
    //         * @param inertiaTensor A pointer to a matrix to hold the
    //         * current inertia tensor of the rigid body. The inertia
    //         * tensor is expressed in the rigid body's local space.
    //         */
    //        void getInertiaTensor(Matrix3 *inertiaTensor) const;
    //
    //        /**
    //         * Gets a copy of the current inertia tensor of the rigid body.
    //         *
    //         * @return A new matrix containing the current intertia
    //         * tensor. The inertia tensor is expressed in the rigid body's
    //         * local space.
    //         */
    //        Matrix3 getInertiaTensor() const;
    //
    //        /**
    //         * Copies the current inertia tensor of the rigid body into
    //         * the given matrix.
    //         *
    //         * @param inertiaTensor A pointer to a matrix to hold the
    //         * current inertia tensor of the rigid body. The inertia
    //         * tensor is expressed in world space.
    //         */
    //        void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;
    //
    //        /**
    //         * Gets a copy of the current inertia tensor of the rigid body.
    //         *
    //         * @return A new matrix containing the current intertia
    //         * tensor. The inertia tensor is expressed in world space.
    //         */
    //        Matrix3 getInertiaTensorWorld() const;
    //
    //        /**
    //         * Sets the inverse intertia tensor for the rigid body.
    //         *
    //         * @param inverseInertiaTensor The inverse inertia tensor for
    //         * the rigid body. This must be a full rank matrix and must be
    //         * invertible.
    //         *
    //         * @warning This invalidates internal data for the rigid body.
    //         * Either an integration function, or the calculateInternals
    //         * function should be called before trying to get any settings
    //         * from the rigid body.
    //         */
    //        void setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);
    //
    //        /**
    //         * Copies the current inverse inertia tensor of the rigid body
    //         * into the given matrix.
    //         *
    //         * @param inverseInertiaTensor A pointer to a matrix to hold
    //         * the current inverse inertia tensor of the rigid body. The
    //         * inertia tensor is expressed in the rigid body's local
    //         * space.
    //         */
    //        void getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;
    //
    //        /**
    //         * Gets a copy of the current inverse inertia tensor of the
    //         * rigid body.
    //         *
    //         * @return A new matrix containing the current inverse
    //         * intertia tensor. The inertia tensor is expressed in the
    //         * rigid body's local space.
    //         */
    //        Matrix3 getInverseInertiaTensor() const;

            (**
             * Copies the current inverse inertia tensor of the rigid body
             * into the given matrix.
             *
             * @param inverseInertiaTensor A pointer to a matrix to hold
             * the current inverse inertia tensor of the rigid body. The
             * inertia tensor is expressed in world space.
             *)
    Procedure getInverseInertiaTensorWorld(Out _inverseInertiaTensor: Matrix3);

    (**
     * Gets a copy of the current inverse inertia tensor of the
     * rigid body.
     *
     * @return A new matrix containing the current inverse
     * intertia tensor. The inertia tensor is expressed in world
     * space.
     *)
    Function getInverseInertiaTensorWorld(): Matrix3;

    (**
     * Sets both linear and angular damping in one function call.
     *
     * @param linearDamping The speed that velocity is shed from
     * the rigid body.
     *
     * @param angularDamping The speed that rotation is shed from
     * the rigid body.
     *
     * @see setLinearDamping
     * @see setAngularDamping
     *)
    Procedure setDamping(Const alinearDamping: Float; Const aangularDamping: Float);

    (**
     * Sets the linear damping for the rigid body.
     *
     * @param linearDamping The speed that velocity is shed from
     * the rigid body.
     *
     * @see setAngularDamping
     *)
    Procedure setLinearDamping(Const alinearDamping: float);

    //        /**
    //         * Gets the current linear damping value.
    //         *
    //         * @return The current linear damping value.
    //         */
    //        real getLinearDamping() const;
    //
            (**
             * Sets the angular damping for the rigid body.
             *
             * @param angularDamping The speed that rotation is shed from
             * the rigid body.
             *
             * @see setLinearDamping
             *)
    Procedure setAngularDamping(Const aangularDamping: Float);
    //
    //        /**
    //         * Gets the current angular damping value.
    //         *
    //         * @return The current angular damping value.
    //         */
    //        real getAngularDamping() const;

        (**
         * Sets the position of the rigid body.
         *
         * @param position The new position of the rigid body.
         *)
    Procedure setPosition(Const aposition: Vector3);

    //        /**
    //         * Sets the position of the rigid body by component.
    //         *
    //         * @param x The x coordinate of the new position of the rigid
    //         * body.
    //         *
    //         * @param y The y coordinate of the new position of the rigid
    //         * body.
    //         *
    //         * @param z The z coordinate of the new position of the rigid
    //         * body.
    //         */
    //        void setPosition(const real x, const real y, const real z);
    //
            (**
             * Fills the given vector with the position of the rigid body.
             *
             * @param position A pointer to a vector into which to write
             * the position.
             *)
    Procedure getPosition(Out aposition: Vector3);

    (**
     * Gets the position of the rigid body.
     *
     * @return The position of the rigid body.
     *)
    Function getPosition(): Vector3;

    (**
     * Sets the orientation of the rigid body.
     *
     * @param orientation The new orientation of the rigid body.
     *
     * @note The given orientation does not need to be normalised,
     * and can be zero. This function automatically constructs a
     * valid rotation quaternion with (0,0,0,0) mapping to
     * (1,0,0,0).
     *)
    Procedure setOrientation(Const aorientation: Quaternion);

    //        /**
    //         * Sets the orientation of the rigid body by component.
    //         *
    //         * @param r The real component of the rigid body's orientation
    //         * quaternion.
    //         *
    //         * @param i The first complex component of the rigid body's
    //         * orientation quaternion.
    //         *
    //         * @param j The second complex component of the rigid body's
    //         * orientation quaternion.
    //         *
    //         * @param k The third complex component of the rigid body's
    //         * orientation quaternion.
    //         *
    //         * @note The given orientation does not need to be normalised,
    //         * and can be zero. This function automatically constructs a
    //         * valid rotation quaternion with (0,0,0,0) mapping to
    //         * (1,0,0,0).
    //         */
    //        void setOrientation(const real r, const real i,
    //            const real j, const real k);

            (**
             * Fills the given quaternion with the current value of the
             * rigid body's orientation.
             *
             * @param orientation A pointer to a quaternion to receive the
             * orientation data.
             *)
    Procedure getOrientation(Out aorientation: Quaternion);

    //        /**
    //         * Gets the orientation of the rigid body.
    //         *
    //         * @return The orientation of the rigid body.
    //         */
    //        Quaternion getOrientation() const;
    //
    //        /**
    //         * Fills the given matrix with a transformation representing
    //         * the rigid body's orientation.
    //         *
    //         * @note Transforming a direction vector by this matrix turns
    //         * it from the body's local space to world space.
    //         *
    //         * @param matrix A pointer to the matrix to fill.
    //         */
    //        void getOrientation(Matrix3 *matrix) const;
    //
    //        /**
    //         * Fills the given matrix data structure with a transformation
    //         * representing the rigid body's orientation.
    //         *
    //         * @note Transforming a direction vector by this matrix turns
    //         * it from the body's local space to world space.
    //         *
    //         * @param matrix A pointer to the matrix to fill.
    //         */
    //        void getOrientation(real matrix[9]) const;
    //
            (**
             * Fills the given matrix with a transformation representing
             * the rigid body's position and orientation.
             *
             * @note Transforming a vector by this matrix turns it from
             * the body's local space to world space.
             *
             * @param transform A pointer to the matrix to fill.
             *)
    Procedure getTransform(Out transform: Matrix4); overload;

    //        /**
    //         * Fills the given matrix data structure with a
    //         * transformation representing the rigid body's position and
    //         * orientation.
    //         *
    //         * @note Transforming a vector by this matrix turns it from
    //         * the body's local space to world space.
    //         *
    //         * @param matrix A pointer to the matrix to fill.
    //         */
    //        void getTransform(real matrix[16]) const;
    //
            (**
             * Fills the given matrix data structure with a
             * transformation representing the rigid body's position and
             * orientation. The matrix is transposed from that returned
             * by getTransform. This call returns a matrix suitable
             * for applying as an OpenGL transform.
             *
             * @note Transforming a vector by this matrix turns it from
             * the body's local space to world space.
             *
             * @param matrix A pointer to the matrix to fill.
             *)
    Procedure getGLTransform(Out matrix: TOpenGLMatrix);

    (**
     * Gets a transformation representing the rigid body's
     * position and orientation.
     *
     * @note Transforming a vector by this matrix turns it from
     * the body's local space to world space.
     *
     * @return The transform matrix for the rigid body.
     *)
    Function getTransform(): Matrix4; overload;
    //
    //        /**
    //         * Converts the given point from world space into the body's
    //         * local space.
    //         *
    //         * @param point The point to covert, given in world space.
    //         *
    //         * @return The converted point, in local space.
    //         */
    //        Vector3 getPointInLocalSpace(const Vector3 &point) const;
    //
            (**
             * Converts the given point from world space into the body's
             * local space.
             *
             * @param point The point to covert, given in local space.
             *
             * @return The converted point, in world space.
             *)
    Function getPointInWorldSpace(Const point: PVector3): Vector3;
    //
    //        /**
    //         * Converts the given direction from world space into the
    //         * body's local space.
    //         *
    //         * @note When a direction is converted between frames of
    //         * reference, there is no translation required.
    //         *
    //         * @param direction The direction to covert, given in world
    //         * space.
    //         *
    //         * @return The converted direction, in local space.
    //         */
    //        Vector3 getDirectionInLocalSpace(const Vector3 &direction) const;
    //
    //        /**
    //         * Converts the given direction from world space into the
    //         * body's local space.
    //         *
    //         * @note When a direction is converted between frames of
    //         * reference, there is no translation required.
    //         *
    //         * @param direction The direction to covert, given in local
    //         * space.
    //         *
    //         * @return The converted direction, in world space.
    //         */
    //        Vector3 getDirectionInWorldSpace(const Vector3 &direction) const;

            (**
             * Sets the velocity of the rigid body.
             *
             * @param velocity The new velocity of the rigid body. The
             * velocity is given in world space.
             *)
    Procedure setVelocity(Const avelocity: Vector3);

    //        /**
    //         * Sets the velocity of the rigid body by component. The
    //         * velocity is given in world space.
    //         *
    //         * @param x The x coordinate of the new velocity of the rigid
    //         * body.
    //         *
    //         * @param y The y coordinate of the new velocity of the rigid
    //         * body.
    //         *
    //         * @param z The z coordinate of the new velocity of the rigid
    //         * body.
    //         */
    //        void setVelocity(const real x, const real y, const real z);

            (**
             * Fills the given vector with the velocity of the rigid body.
             *
             * @param velocity A pointer to a vector into which to write
             * the velocity. The velocity is given in world local space.
             *)
    Procedure getVelocity(Var avelocity: Vector3);

    (**
     * Gets the velocity of the rigid body.
     *
     * @return The velocity of the rigid body. The velocity is
     * given in world local space.
     *)
    Function getVelocity(): Vector3;

    (**
     *Applies the given change in velocity.
     *)
    Procedure addVelocity(Const deltaVelocity: Vector3);

    (**
     * Sets the rotation of the rigid body.
     *
     * @param rotation The new rotation of the rigid body. The
     * rotation is given in world space.
     *)
    Procedure setRotation(Const arotation: Vector3);

    //        /**
    //         * Sets the rotation of the rigid body by component. The
    //         * rotation is given in world space.
    //         *
    //         * @param x The x coordinate of the new rotation of the rigid
    //         * body.
    //         *
    //         * @param y The y coordinate of the new rotation of the rigid
    //         * body.
    //         *
    //         * @param z The z coordinate of the new rotation of the rigid
    //         * body.
    //         */
    //        void setRotation(const real x, const real y, const real z);

            (**
             * Fills the given vector with the rotation of the rigid body.
             *
             * @param rotation A pointer to a vector into which to write
             * the rotation. The rotation is given in world local space.
             *)
    Procedure getRotation(Var arotation: Vector3);

    (**
     * Gets the rotation of the rigid body.
     *
     * @return The rotation of the rigid body. The rotation is
     * given in world local space.
     *)
    Function getRotation(): Vector3;

    (**
     * Applies the given change in rotation.
     *)
    Procedure addRotation(Const deltaRotation: Vector3);

    (**
     * Returns true if the body is awake and responding to
     * integration.
     *
     * @return The awake state of the body.
     *)
    Function getAwake(): boolean;

    (**
     * Sets the awake state of the body. If the body is set to be
     * not awake, then its velocities are also cancelled, since
     * a moving body that is not awake can cause problems in the
     * simulation.
     *
     * @param awake The new awake state of the body.
     *)
    Procedure setAwake(awake: boolean = true);

    //        /**
    //         * Returns true if the body is allowed to go to sleep at
    //         * any time.
    //         */
    //        bool getCanSleep() const
    //        {
    //            return canSleep;
    //        }

            (**
             * Sets whether the body is ever allowed to go to sleep. Bodies
             * under the player's control, or for which the set of
             * transient forces applied each frame are not predictable,
             * should be kept awake.
             *
             * @param canSleep Whether the body can now be put to sleep.
             *)
    Procedure setCanSleep(Const acanSleep: boolean);

    (*@}*)


    (**
     * @name Retrieval Functions for Dynamic Quantities
     *
     * These functions provide access to the acceleration
     * properties of the body. The acceleration is generated by
     * the simulation from the forces and torques applied to the
     * rigid body. Acceleration cannot be directly influenced, it
     * is set during integration, and represent the acceleration
     * experienced by the body of the previous simulation step.
     *)
    (*@{*)

    (**
     * Fills the given vector with the current accumulated value
     * for linear acceleration. The acceleration accumulators
     * are set during the integration step. They can be read to
     * determine the rigid body's acceleration over the last
     * integration step. The linear acceleration is given in world
     * space.
     *
     * @param linearAcceleration A pointer to a vector to receive
     * the linear acceleration data.
     *)
    Procedure getLastFrameAcceleration(Var linearAcceleration: Vector3);

    (**
     * Gets the current accumulated value for linear
     * acceleration. The acceleration accumulators are set during
     * the integration step. They can be read to determine the
     * rigid body's acceleration over the last integration
     * step. The linear acceleration is given in world space.
     *
     * @return The rigid body's linear acceleration.
     *)
    Function getLastFrameAcceleration(): Vector3;

    (*@}*)


    (**
     * @name Force, Torque and Acceleration Set-up Functions
     *
     * These functions set up forces and torques to apply to the
     * rigid body.
     *)
    (*@{*)

    (**
     * Clears the forces and torques in the accumulators. This will
     * be called automatically after each intergration step.
     *)
    Procedure clearAccumulators();

    //        /**
    //         * Adds the given force to centre of mass of the rigid body.
    //         * The force is expressed in world-coordinates.
    //         *
    //         * @param force The force to apply.
    //         */
    //        void addForce(const Vector3 &force);

            (**
             * Adds the given force to the given point on the rigid body.
             * Both the force and the
             * application point are given in world space. Because the
             * force is not applied at the centre of mass, it may be split
             * into both a force and torque.
             *
             * @param force The force to apply.
             *
             * @param point The location at which to apply the force, in
             * world-coordinates.
             *)
    Procedure addForceAtPoint(Const force: Vector3; Const point: Vector3);

    (**
     * Adds the given force to the given point on the rigid body.
     * The direction of the force is given in world coordinates,
     * but the application point is given in body space. This is
     * useful for spring forces, or other forces fixed to the
     * body.
     *
     * @param force The force to apply.
     *
     * @param point The location at which to apply the force, in
     * body-coordinates.
     *)
    Procedure addForceAtBodyPoint(Const force: Vector3; Const point: Vector3);

    //        /**
    //         * Adds the given torque to the rigid body.
    //         * The force is expressed in world-coordinates.
    //         *
    //         * @param torque The torque to apply.
    //         */
    //        void addTorque(const Vector3 &torque);
    //
            (**
             * Sets the constant acceleration of the rigid body.
             *
             * @param acceleration The new acceleration of the rigid body.
             *)
    Procedure setAcceleration(Const aacceleration: Vector3);

    //        /**
    //         * Sets the constant acceleration of the rigid body by component.
    //         *
    //         * @param x The x coordinate of the new acceleration of the rigid
    //         * body.
    //         *
    //         * @param y The y coordinate of the new acceleration of the rigid
    //         * body.
    //         *
    //         * @param z The z coordinate of the new acceleration of the rigid
    //         * body.
    //         */
    //        void setAcceleration(const real x, const real y, const real z);
    //
    //        /**
    //         * Fills the given vector with the acceleration of the rigid body.
    //         *
    //         * @param acceleration A pointer to a vector into which to write
    //         * the acceleration. The acceleration is given in world local space.
    //         */
    //        void getAcceleration(Vector3 *acceleration) const;
    //
    //        /**
    //         * Gets the acceleration of the rigid body.
    //         *
    //         * @return The acceleration of the rigid body. The acceleration is
    //         * given in world local space.
    //         */
    //        Vector3 getAcceleration() const;
    //
    //        /*@}*/

  End;

  PRigidBody = ^RigidBody;

Implementation

(**
 * Internal function that checks the validity of an inverse inertia tensor.
 *)

Procedure _checkInverseInertiaTensor(Const iitWorld: Matrix3);
Begin
  // TODO: Perform a validity check in an assert.
End;

(**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 *)

Procedure _transformInertiaTensor(Var iitWorld: Matrix3;
  Const q: Quaternion;
  Const iitBody: Matrix3;
  Const rotmat: Matrix4);
Var
  t4, t9, t14, t28, t33, t38, t52, t57, t62: Float;
Begin
  t4 := rotmat.data[0] * iitBody.data[0] +
    rotmat.data[1] * iitBody.data[3] +
    rotmat.data[2] * iitBody.data[6];
  t9 := rotmat.data[0] * iitBody.data[1] +
    rotmat.data[1] * iitBody.data[4] +
    rotmat.data[2] * iitBody.data[7];
  t14 := rotmat.data[0] * iitBody.data[2] +
    rotmat.data[1] * iitBody.data[5] +
    rotmat.data[2] * iitBody.data[8];
  t28 := rotmat.data[4] * iitBody.data[0] +
    rotmat.data[5] * iitBody.data[3] +
    rotmat.data[6] * iitBody.data[6];
  t33 := rotmat.data[4] * iitBody.data[1] +
    rotmat.data[5] * iitBody.data[4] +
    rotmat.data[6] * iitBody.data[7];
  t38 := rotmat.data[4] * iitBody.data[2] +
    rotmat.data[5] * iitBody.data[5] +
    rotmat.data[6] * iitBody.data[8];
  t52 := rotmat.data[8] * iitBody.data[0] +
    rotmat.data[9] * iitBody.data[3] +
    rotmat.data[10] * iitBody.data[6];
  t57 := rotmat.data[8] * iitBody.data[1] +
    rotmat.data[9] * iitBody.data[4] +
    rotmat.data[10] * iitBody.data[7];
  t62 := rotmat.data[8] * iitBody.data[2] +
    rotmat.data[9] * iitBody.data[5] +
    rotmat.data[10] * iitBody.data[8];

  iitWorld.data[0] := t4 * rotmat.data[0] +
    t9 * rotmat.data[1] +
    t14 * rotmat.data[2];
  iitWorld.data[1] := t4 * rotmat.data[4] +
    t9 * rotmat.data[5] +
    t14 * rotmat.data[6];
  iitWorld.data[2] := t4 * rotmat.data[8] +
    t9 * rotmat.data[9] +
    t14 * rotmat.data[10];
  iitWorld.data[3] := t28 * rotmat.data[0] +
    t33 * rotmat.data[1] +
    t38 * rotmat.data[2];
  iitWorld.data[4] := t28 * rotmat.data[4] +
    t33 * rotmat.data[5] +
    t38 * rotmat.data[6];
  iitWorld.data[5] := t28 * rotmat.data[8] +
    t33 * rotmat.data[9] +
    t38 * rotmat.data[10];
  iitWorld.data[6] := t52 * rotmat.data[0] +
    t57 * rotmat.data[1] +
    t62 * rotmat.data[2];
  iitWorld.data[7] := t52 * rotmat.data[4] +
    t57 * rotmat.data[5] +
    t62 * rotmat.data[6];
  iitWorld.data[8] := t52 * rotmat.data[8] +
    t57 * rotmat.data[9] +
    t62 * rotmat.data[10];
End;

(**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 *)

Procedure _calculateTransformMatrix(Var transformMatrix: Matrix4;
  Const position: Vector3;
  Const orientation: Quaternion);
Begin
  transformMatrix.data[0] := 1 - 2 * orientation._d.j * orientation._d.j -
    2 * orientation._d.k * orientation._d.k;
  transformMatrix.data[1] := 2 * orientation._d.i * orientation._d.j -
    2 * orientation._d.r * orientation._d.k;
  transformMatrix.data[2] := 2 * orientation._d.i * orientation._d.k +
    2 * orientation._d.r * orientation._d.j;
  transformMatrix.data[3] := position.x;

  transformMatrix.data[4] := 2 * orientation._d.i * orientation._d.j +
    2 * orientation._d.r * orientation._d.k;
  transformMatrix.data[5] := 1 - 2 * orientation._d.i * orientation._d.i -
    2 * orientation._d.k * orientation._d.k;
  transformMatrix.data[6] := 2 * orientation._d.j * orientation._d.k -
    2 * orientation._d.r * orientation._d.i;
  transformMatrix.data[7] := position.y;

  transformMatrix.data[8] := 2 * orientation._d.i * orientation._d.k -
    2 * orientation._d.r * orientation._d.j;
  transformMatrix.data[9] := 2 * orientation._d.j * orientation._d.k +
    2 * orientation._d.r * orientation._d.i;
  transformMatrix.data[10] := 1 - 2 * orientation._d.i * orientation._d.i -
    2 * orientation._d.j * orientation._d.j;
  transformMatrix.data[11] := position.z;
End;

{ RigidBody }

Constructor RigidBody.Create;
Begin
  Inherited create;
  isAwake := false;
  transformMatrix.create();
  orientation.Create();
  velocity.create();
  rotation.create();
  inverseInertiaTensor.create();
  inverseInertiaTensorWorld.create();
  motion := 0;
End;

Destructor RigidBody.Destroy();
Begin
  Nop(); // Nur zum Debuggen ;)
End;

Procedure RigidBody.calculateDerivedData;
Begin
  orientation.normalise();

  // Calculate the transform matrix for the body.
  _calculateTransformMatrix(transformMatrix, position, orientation);

  // Calculate the inertiaTensor in world space.
  _transformInertiaTensor(inverseInertiaTensorWorld,
    orientation,
    inverseInertiaTensor,
    transformMatrix);
End;

Procedure RigidBody.integrate(duration: float);
Var
  angularAcceleration: Vector3;
  currentMotion, bias: float;
Begin
  If (Not isAwake) Then exit;

  // Calculate linear acceleration from force inputs.
  lastFrameAcceleration := acceleration;
  lastFrameAcceleration.addScaledVector(forceAccum, inverseMass);

  // Calculate angular acceleration from torque inputs.
  angularAcceleration := inverseInertiaTensorWorld.transform(torqueAccum);

  // Adjust velocities
  // Update linear velocity from both acceleration and impulse.
  velocity.addScaledVector(lastFrameAcceleration, duration);

  // Update angular velocity from both acceleration and impulse.
  rotation.addScaledVector(angularAcceleration, duration);

  // Impose drag.
  velocity := velocity * real_pow(linearDamping, duration);
  rotation := rotation * real_pow(angularDamping, duration);

  // Adjust positions
  // Update linear position.
  position.addScaledVector(velocity, duration);

  // Update angular position.
  orientation.addScaledVector(rotation, duration);

  // Normalise the orientation, and update the matrices with the new
  // position and orientation
  calculateDerivedData();

  // Clear accumulators.
  clearAccumulators();

  // Update the kinetic energy store, and possibly put the body to
  // sleep.
  If (canSleep) Then Begin
    currentMotion := velocity.scalarProduct(velocity) +
      rotation.scalarProduct(rotation);

    bias := real_pow(0.5, duration);
    motion := bias * motion + (1 - bias) * currentMotion;

    If (motion < sleepEpsilon) Then Begin
      setAwake(false);
    End
    Else If (motion > 10 * sleepEpsilon) Then
      motion := 10 * sleepEpsilon;
  End;
End;

Procedure RigidBody.setMass(Const mass: float);
Begin
  assert(mass <> 0);
  inverseMass := (1.0) / mass;
End;

Function RigidBody.getInverseMass: float;
Begin
  result := inverseMass;
End;

Procedure RigidBody.setInertiaTensor(Const inertiaTensor: Matrix3);
Begin
  inverseInertiaTensor.setInverse(inertiaTensor);
  _checkInverseInertiaTensor(inverseInertiaTensor);
End;

Procedure RigidBody.getInverseInertiaTensorWorld(Out
  _inverseInertiaTensor: Matrix3);
Begin
  _inverseInertiaTensor := inverseInertiaTensorWorld;
End;

Function RigidBody.getInverseInertiaTensorWorld: Matrix3;
Begin
  result := inverseInertiaTensorWorld;
End;

Procedure RigidBody.setDamping(Const alinearDamping: Float;
  Const aangularDamping: Float);
Begin
  linearDamping := alinearDamping;
  angularDamping := aangularDamping;
End;

Procedure RigidBody.setLinearDamping(Const alinearDamping: float);
Begin
  linearDamping := alinearDamping;
End;

Procedure RigidBody.setAngularDamping(Const aangularDamping: Float);
Begin
  angularDamping := aangularDamping;
End;

Procedure RigidBody.setPosition(Const aposition: Vector3);
Begin
  position := aposition;
End;

Procedure RigidBody.getPosition(Out aposition: Vector3);
Begin
  aposition := position;
End;

Function RigidBody.getPosition: Vector3;
Begin
  result := position;
End;

Procedure RigidBody.setOrientation(Const aorientation: Quaternion);
Begin
  orientation := aorientation;
  orientation.normalise();
End;

Procedure RigidBody.getOrientation(Out aorientation: Quaternion);
Begin
  aorientation := orientation;
End;

Procedure RigidBody.getTransform(Out transform: Matrix4);
Var
  i: Integer;
Begin
  For i := 0 To high(transformMatrix.data) Do
    transform.data[i] := transformMatrix.data[i];
End;

Procedure RigidBody.getGLTransform(Out matrix: TOpenGLMatrix);
Begin
  matrix[0] := transformMatrix.data[0];
  matrix[1] := transformMatrix.data[4];
  matrix[2] := transformMatrix.data[8];
  matrix[3] := 0;

  matrix[4] := transformMatrix.data[1];
  matrix[5] := transformMatrix.data[5];
  matrix[6] := transformMatrix.data[9];
  matrix[7] := 0;

  matrix[8] := transformMatrix.data[2];
  matrix[9] := transformMatrix.data[6];
  matrix[10] := transformMatrix.data[10];
  matrix[11] := 0;

  matrix[12] := transformMatrix.data[3];
  matrix[13] := transformMatrix.data[7];
  matrix[14] := transformMatrix.data[11];
  matrix[15] := 1;
End;

Function RigidBody.getTransform: Matrix4;
Begin
  result := transformMatrix;
End;

Function RigidBody.getPointInWorldSpace(Const point: PVector3): Vector3;
Begin
  result := transformMatrix.transform(point^);
End;

Procedure RigidBody.setVelocity(Const avelocity: Vector3);
Begin
  velocity := avelocity;
End;

Procedure RigidBody.getVelocity(Var avelocity: Vector3);
Begin
  avelocity := velocity;
End;

Function RigidBody.getVelocity: Vector3;
Begin
  result := velocity;
End;

Procedure RigidBody.addVelocity(Const deltaVelocity: Vector3);
Begin
  velocity := velocity + deltaVelocity;
End;

Procedure RigidBody.setRotation(Const arotation: Vector3);
Begin
  rotation := arotation;
End;

Procedure RigidBody.getRotation(Var arotation: Vector3);
Begin
  arotation := rotation;
End;

Function RigidBody.getRotation: Vector3;
Begin
  result := rotation;
End;

Procedure RigidBody.addRotation(Const deltaRotation: Vector3);
Begin
  rotation := rotation + deltaRotation;
End;

Function RigidBody.getAwake: boolean;
Begin
  result := isAwake;
End;

Procedure RigidBody.setAwake(awake: boolean);
Begin
  If (awake) Then Begin
    isAwake := true;

    // Add a bit of motion to avoid it falling asleep immediately.
    motion := sleepEpsilon * 2.0;
  End
  Else Begin
    isAwake := false;
    velocity.clear();
    rotation.clear();
  End;
End;

Procedure RigidBody.setCanSleep(Const acanSleep: boolean);
Begin
  canSleep := acanSleep;
  If (Not canSleep) And (Not isAwake) Then setAwake();
End;

Procedure RigidBody.getLastFrameAcceleration(Var linearAcceleration: Vector3);
Begin
  acceleration := lastFrameAcceleration;
End;

Function RigidBody.getLastFrameAcceleration: Vector3;
Begin
  result := lastFrameAcceleration;
End;

Procedure RigidBody.clearAccumulators;
Begin
  forceAccum.clear();
  torqueAccum.clear();
End;

Procedure RigidBody.addForceAtPoint(Const force: Vector3; Const point: Vector3);
Var
  pt: Vector3;
Begin
  // Convert to coordinates relative to center of mass.
  pt := point;
  pt := pt - position;

  forceAccum := forceAccum + force;
  torqueAccum := torqueAccum + (pt Mod force);

  isAwake := true;
End;

Procedure RigidBody.addForceAtBodyPoint(Const force: Vector3;
  Const point: Vector3);
Var
  pt: Vector3;
Begin
  // Convert to coordinates relative to center of mass.
  pt := getPointInWorldSpace(@point);
  addForceAtPoint(force, pt);
End;

Procedure RigidBody.setAcceleration(Const aacceleration: Vector3);
Begin
  acceleration := aacceleration;
End;

End.

