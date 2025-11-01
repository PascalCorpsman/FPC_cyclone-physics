Unit ufgen;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ubody, uprecision, ucore;

Type

  (**
   * A force generator can be asked to add a force to one or more
   * bodies.
   *)
  ForceGenerator = Class
  public
    (**
     * Overload this in implementations of the interface to calculate
     * and update the force applied to the given rigid body.
     *)
    Procedure updateForce(Var body: RigidBody; duration: float); virtual; abstract;
  End;

  PForceGenerator = ^ForceGenerator;

  (**
   * A force generator that applies a gravitational force. One instance
   * can be used for multiple rigid bodies.
   *)
//    class Gravity : public ForceGenerator
//    {
//        /** Holds the acceleration due to gravity. */
//        Vector3 gravity;
//
//    public:
//
//        /** Creates the generator with the given acceleration. */
//        Gravity(const Vector3 &gravity);
//
//        /** Applies the gravitational force to the given rigid body. */
//        virtual void updateForce(RigidBody *body, real duration);
//    };
//
//    /**
//     * A force generator that applies a Spring force.
//     */
//    class Spring : public ForceGenerator
//    {
//        /**
//         * The point of connection of the spring, in local
//         * coordinates.
//         */
//        Vector3 connectionPoint;
//
//        /**
//         * The point of connection of the spring to the other object,
//         * in that object's local coordinates.
//         */
//        Vector3 otherConnectionPoint;
//
//        /** The particle at the other end of the spring. */
//        RigidBody *other;
//
//        /** Holds the sprint constant. */
//        real springConstant;
//
//        /** Holds the rest length of the spring. */
//        real restLength;
//
//    public:
//
//        /** Creates a new spring with the given parameters. */
//        Spring(const Vector3 &localConnectionPt,
//               RigidBody *other,
//               const Vector3 &otherConnectionPt,
//               real springConstant,
//               real restLength);
//
//        /** Applies the spring force to the given rigid body. */
//        virtual void updateForce(RigidBody *body, real duration);
//    };
//
//    /**
//     * A force generator showing a three component explosion effect.
//     * This force generator is intended to represent a single
//     * explosion effect for multiple rigid bodies. The force generator
//     * can also act as a particle force generator.
//     */
//    class Explosion : public ForceGenerator,
//                      public ParticleForceGenerator
//    {
//        /**
//         * Tracks how long the explosion has been in operation, used
//         * for time-sensitive effects.
//         */
//        real timePassed;
//
//    public:
//        // Properties of the explosion, these are public because
//        // there are so many and providing a suitable constructor
//        // would be cumbersome:
//
//        /**
//         * The location of the detonation of the weapon.
//         */
//        Vector3 detonation;
//
//        // ... Other Explosion code as before ...
//
//
//        /**
//         * The radius up to which objects implode in the first stage
//         * of the explosion.
//         */
//        real implosionMaxRadius;
//
//        /**
//         * The radius within which objects don't feel the implosion
//         * force. Objects near to the detonation aren't sucked in by
//         * the air implosion.
//         */
//        real implosionMinRadius;
//
//        /**
//         * The length of time that objects spend imploding before the
//         * concussion phase kicks in.
//         */
//        real implosionDuration;
//
//        /**
//         * The maximal force that the implosion can apply. This should
//         * be relatively small to avoid the implosion pulling objects
//         * through the detonation point and out the other side before
//         * the concussion wave kicks in.
//         */
//        real implosionForce;
//
//        /**
//         * The speed that the shock wave is traveling, this is related
//         * to the thickness below in the relationship:
//         *
//         * thickness >= speed * minimum frame duration
//         */
//        real shockwaveSpeed;
//
//        /**
//         * The shock wave applies its force over a range of distances,
//         * this controls how thick. Faster waves require larger
//         * thicknesses.
//         */
//        real shockwaveThickness;
//
//        /**
//         * This is the force that is applied at the very centre of the
//         * concussion wave on an object that is stationary. Objects
//         * that are in front or behind of the wavefront, or that are
//         * already moving outwards, get proportionally less
//         * force. Objects moving in towards the centre get
//         * proportionally more force.
//         */
//         real peakConcussionForce;
//
//         /**
//          * The length of time that the concussion wave is active.
//          * As the wave nears this, the forces it applies reduces.
//          */
//         real concussionDuration;
//
//         /**
//          * This is the peak force for stationary objects in
//          * the centre of the convection chimney. Force calculations
//          * for this value are the same as for peakConcussionForce.
//          */
//         real peakConvectionForce;
//
//         /**
//          * The radius of the chimney cylinder in the xz plane.
//          */
//         real chimneyRadius;
//
//         /**
//          * The maximum height of the chimney.
//          */
//         real chimneyHeight;
//
//         /**
//          * The length of time the convection chimney is active. Typically
//          * this is the longest effect to be in operation, as the heat
//          * from the explosion outlives the shock wave and implosion
//          * itself.
//          */
//         real convectionDuration;
//
//    public:
//        /**
//         * Creates a new explosion with sensible default values.
//         */
//        Explosion();
//
//        /**
//         * Calculates and applies the force that the explosion
//         * has on the given rigid body.
//         */
//        virtual void updateForce(RigidBody * body, real duration);
//
//        /**
//         * Calculates and applies the force that the explosion has
//         * on the given particle.
//         */
//        virtual void updateForce(Particle *particle, real duration) = 0;
//
//    };

    (**
     * A force generator that applies an aerodynamic force.
     *)

  { Aero }

  Aero = Class(ForceGenerator)
  protected
    (**
     * Holds the aerodynamic tensor for the surface in body
     * space.
     *)
    tensor: Matrix3;

    (**
     * Holds the relative position of the aerodynamic surface in
     * body coordinates.
     *)
    position: Vector3;

    (**
     * Holds a pointer to a vector containing the windspeed of the
     * environment. This is easier than managing a separate
     * windspeed vector per generator and having to update it
     * manually as the wind changes.
     *)
    windspeed: PVector3;

  public
    (**
     * Creates a new aerodynamic force generator with the
     * given properties.
     *)
    Constructor Create(Const atensor: Matrix3; Const aposition: Vector3;
      Const awindspeed: PVector3); virtual;

    (**
     * Applies the force to the given rigid body.
     *)
    Procedure updateForce(Var body: RigidBody; duration: float); override;
  protected
    (**
     * Uses an explicit tensor matrix to update the force on
     * the given rigid body. This is exactly the same as for updateForce
     * only it takes an explicit tensor.
     *)
    Procedure updateForceFromTensor(Var body: RigidBody; duration: real;
      Const atensor: Matrix3);
  End;

  (**
   * A force generator with a control aerodynamic surface. This
   * requires three inertia tensors, for the two extremes and
   * 'resting' position of the control surface.  The latter tensor is
   * the one inherited from the base class, the two extremes are
   * defined in this class.
   *)

  { AeroControl }

  AeroControl = Class(Aero)
  protected
    (**
     * The aerodynamic tensor for the surface, when the control is at
     * its maximum value.
     *)
    maxTensor: Matrix3;

    (**
     * The aerodynamic tensor for the surface, when the control is at
     * its minimum value.
     *)
    minTensor: Matrix3;

    (**
     * The current position of the control for this surface. This
     * should range between -1 (in which case the minTensor value
     * is used), through 0 (where the base-class tensor value is
     * used) to +1 (where the maxTensor value is used).
     *)
    controlSetting: float;
  private
    (**
     * Calculates the final aerodynamic tensor for the current
     * control setting.
     *)
    Function getTensor(): Matrix3;

  public
    (**
     * Creates a new aerodynamic control surface with the given
     * properties.
     *)

    Constructor Create(Const abase: Matrix3;
      Const amin, amax: Matrix3;
      Const aposition: Vector3; Const awindspeed: PVector3); virtual; reintroduce;

    (**
     * Sets the control position of this control. This
     * should range between -1 (in which case the minTensor value is
     * used), through 0 (where the base-class tensor value is used)
     * to +1 (where the maxTensor value is used). Values outside that
     * range give undefined results.
     *)
    Procedure setControl(value: Float);

    (**
     * Applies the force to the given rigid body.
     *)
    Procedure updateForce(Var body: RigidBody; duration: float); override;
  End;

  //    /**
  //     * A force generator with an aerodynamic surface that can be
  //     * re-oriented relative to its rigid body. This derives the
  //     */
  //    class AngledAero : public Aero
  //    {
  //        /**
  //         * Holds the orientation of the aerodynamic surface relative
  //         * to the rigid body to which it is attached.
  //         */
  //        Quaternion orientation;
  //
  //    public:
  //        /**
  //         * Creates a new aerodynamic surface with the given properties.
  //         */
  //        AngledAero(const Matrix3 &tensor, const Vector3 &position,
  //             const Vector3 *windspeed);
  //
  //        /**
  //         * Sets the relative orientation of the aerodynamic surface,
  //         * relative to the rigid body it is attached to. Note that
  //         * this doesn't affect the point of connection of the surface
  //         * to the body.
  //         */
  //        void setOrientation(const Quaternion &quat);
  //
  //        /**
  //         * Applies the force to the given rigid body.
  //         */
  //        virtual void updateForce(RigidBody *body, real duration);
  //    };

      (**
       * A force generator to apply a buoyant force to a rigid body.
       *)

  { Buoyancy }

  Buoyancy = Class(ForceGenerator)
    (**
     * The maximum submersion depth of the object before
     * it generates its maximum buoyancy force.
     *)
    maxDepth: float;

    (**
     * The volume of the object.
     *)
    volume: Float;

    (**
     * The height of the water plane above y=0. The plane will be
     * parallel to the XZ plane.
     *)
    waterHeight: Float;

    (**
     * The density of the liquid. Pure water has a density of
     * 1000kg per cubic meter.
     *)
    liquidDensity: float;

    (**
     * The centre of buoyancy of the rigid body, in body coordinates.
     *)
    centreOfBuoyancy: Vector3;

  public
    (** Creates a new buoyancy force with the given parameters. *)
    Constructor Create(Const cOfB: Vector3;
      amaxDepth, avolume, awaterHeight: Float;
      aliquidDensity: float = 1000.0);

    (**
     * Applies the force to the given rigid body.
     *)
    Procedure updateForce(Var body: RigidBody; duration: float); override;
  End;

Type
  ForceRegistration = Record // Protected of TRegistry
    body: PRigidBody;
    fg: PForceGenerator;
  End;

  (**
   * Holds the list of registrations.
   *)
  TRegistry = Array Of ForceRegistration;
  (**
   * Holds all the force generators and the bodies they apply to.
   *)

  { ForceRegistry }

  ForceRegistry = Class
  protected

    (**
     * Keeps track of one force generator and the body it
     * applies to.
     *)
    registrations: TRegistry;
  public
    Constructor Create(); virtual;
    Destructor Destroy(); override;
    (**
     * Registers the given force generator to apply to the
     * given body.
     *)
    Procedure add(body: PRigidBody; fg: PForceGenerator);

    (**
     * Removes the given registered pair from the registry.
     * If the pair is not registered, this method will have
     * no effect.
     *)
    Procedure remove(body: PRigidBody; fg: PForceGenerator);

    (**
     * Clears all registrations from the registry. This will
     * not delete the bodies or the force generators
     * themselves, just the records of their connection.
     *)
    Procedure clear();

    (**
     * Calls all the force generators to update the forces of
     * their corresponding bodies.
     *)
    Procedure updateForces(duration: float);
  End;

Implementation

{ Aero }

Constructor Aero.Create(Const atensor: Matrix3; Const aposition: Vector3;
  Const awindspeed: PVector3);
Begin
  tensor := atensor;
  position := aposition;
  windspeed := awindspeed;
End;

Procedure Aero.updateForce(Var body: RigidBody; duration: float);
Begin
  updateForceFromTensor(body, duration, tensor);
End;

Procedure Aero.updateForceFromTensor(Var body: RigidBody; duration: real;
  Const atensor: Matrix3);
Var
  bodyForce, force, velocity, bodyVel: Vector3;
Begin
  // Calculate total velocity (windspeed and body's velocity).
  velocity := body.getVelocity();
  velocity := velocity + windspeed^;

  // Calculate the velocity in body coordinates
  bodyVel := body.getTransform().transformInverseDirection(velocity);

  // Calculate the force in body coordinates
  bodyForce := atensor.transform(bodyVel);
  force := body.getTransform().transformDirection(bodyForce);

  // Apply the force
  body.addForceAtBodyPoint(force, position);
End;

{ AeroControl }

Constructor AeroControl.Create(Const abase: Matrix3; Const amin, amax: Matrix3;
  Const aposition: Vector3; Const awindspeed: PVector3);
Begin
  Inherited create(abase, aposition, awindspeed);
  minTensor := amin;
  maxTensor := amax;
  controlSetting := 0.0;
End;

Function AeroControl.getTensor: Matrix3;
Begin
  If (controlSetting <= -1.0) Then Begin
    result := minTensor;
  End
  Else If (controlSetting >= 1.0) Then Begin
    result := maxTensor;
  End
  Else If (controlSetting < 0) Then Begin
    result := linearInterpolate(minTensor, tensor, controlSetting + 1.0);
  End
  Else If (controlSetting > 0) Then Begin
    result := linearInterpolate(tensor, maxTensor, controlSetting);
  End
  Else
    result := tensor;
End;

Procedure AeroControl.setControl(value: Float);
Begin
  controlSetting := value;
End;

Procedure AeroControl.updateForce(Var body: RigidBody; duration: float);
Var
  atensor: Matrix3;
Begin
  atensor := getTensor();
  updateForceFromTensor(body, duration, atensor);
End;

{ Buoyancy }

Constructor Buoyancy.Create(Const cOfB: Vector3; amaxDepth, avolume,
  awaterHeight: Float; aliquidDensity: float);
Begin
  centreOfBuoyancy := cOfB;
  liquidDensity := aliquidDensity;
  maxDepth := amaxDepth;
  volume := avolume;
  waterHeight := awaterHeight;
End;

Procedure Buoyancy.updateForce(Var body: RigidBody; duration: float);
Var
  force, pointInWorld: Vector3;
  depth: float;
Begin
  // Calculate the submersion depth
  pointInWorld := body.getPointInWorldSpace(centreOfBuoyancy);
  depth := pointInWorld.y;

  // Check if we're out of the water
  If (depth >= waterHeight + maxDepth) Then exit;
  force.create(0, 0, 0);

  // Check if we're at maximum depth
  If (depth <= waterHeight - maxDepth) Then Begin

    force.y := liquidDensity * volume;
    body.addForceAtBodyPoint(force, centreOfBuoyancy);
    exit;
  End;

  // Otherwise we are partly submerged
  force.y := liquidDensity * volume *
    (depth - maxDepth - waterHeight) / 2 * maxDepth;
  body.addForceAtBodyPoint(force, centreOfBuoyancy);
End;

{ ForceRegistry }

Constructor ForceRegistry.Create();
Begin
  Inherited create;
  registrations := Nil;
End;

Destructor ForceRegistry.Destroy();
Begin
  clear;
End;

Procedure ForceRegistry.add(body: PRigidBody; fg: PForceGenerator);
Begin
  setlength(registrations, high(registrations) + 2);
  registrations[high(registrations)].body := body;
  registrations[high(registrations)].fg := fg;
End;

Procedure ForceRegistry.remove(body: PRigidBody; fg: PForceGenerator);
Var
  i, j: Integer;
Begin
  For i := 0 To high(registrations) Do Begin
    If (registrations[i].body = body) And
      (registrations[i].fg = fg) Then Begin
      For j := i To high(registrations) - 1 Do Begin
        registrations[j] := registrations[j + 1];
      End;
      setlength(registrations, high(registrations));
    End;
  End;
End;

Procedure ForceRegistry.clear;
Begin
  setlength(registrations, 0);
End;

Procedure ForceRegistry.updateForces(duration: float);
Var
  i: Integer;
Begin
  For i := 0 To high(registrations) Do Begin
    registrations[i].fg^.updateForce(registrations[i].body^, duration);
  End;
End;

End.

