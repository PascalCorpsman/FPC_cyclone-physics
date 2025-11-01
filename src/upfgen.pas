Unit upfgen;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision, uparticle;

Type

  (**
    * A force generator can be asked to add a force to one or more
    * particles.
    *)
  ParticleForceGenerator = Class
    Procedure updateForce(Var Particle: Particle; duration: float); virtual; abstract;
  End;

  { ParticleGravity }
  (**
   * A force generator that applies a gravitational force. One instance
   * can be used for multiple particles.
   *)
  ParticleGravity = Class(ParticleForceGenerator)
  private
    gravity: Vector3;
  public
    Constructor Create(aGravity: Vector3); virtual;
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleDrag }
  (**
   * A force generator that applies a drag force. One instance
   * can be used for multiple particles.
   *)
  ParticleDrag = Class(ParticleForceGenerator)
  private
    k1, k2: float;
  public
    Constructor Create(ak1, ak2: float); virtual;
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleAnchoredSpring }
  (**
   * A force generator that applies a Spring force, where
   * one end is attached to a fixed point in space.
   *)
  ParticleAnchoredSpring = Class(ParticleForceGenerator)
  protected
    (** The location of the anchored end of the spring. *)
    anchor: PVector3;

    (** Holds the sprint constant. *)
    springConstant: float;

    (** Holds the rest length of the spring. *)
    restLength: float;
  public
    Constructor Create(); virtual;
    (** Creates a new spring with the given parameters. *)
    Constructor ParticleAnchoredSpring(aAnchor: PVector3;
      aSpringConstant, aRestLength: float); virtual;

    Procedure Init(aAnchor: PVector3;
      aSpringConstant, aRestLength: float); virtual;

    Function getAnchor(): PVector3;

    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleAnchoredBungee }
  (**
   * A force generator that applies a bungee force, where
   * one end is attached to a fixed point in space.
   *)
  ParticleAnchoredBungee = Class(ParticleAnchoredSpring)
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleFakeSpring }
  (**
   * A force generator that fakes a stiff spring force, and where
   * one end is attached to a fixed point in space.
   *)
  ParticleFakeSpring = Class(ParticleForceGenerator)
  private

    anchor: PVector3;
    springConstant: float;
    damping: float;

  public

    Constructor ParticleFakeSpring(aAnchor: PVector3; aSpringConstant,
      aDamping: float); virtual;

    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleSpring }
  (**
   * A force generator that applies a Spring force.
   *)
  ParticleSpring = Class(ParticleForceGenerator)
  private
    other: PParticle;
    springConstant: Float;
    restLength: Float;
  public
    Constructor ParticleSpring(aother: PParticle; aspringConstant, arestLength: Float);
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleBungee }
  (**
   * A force generator that applies a spring force only
   * when extended.
   *)
  ParticleBungee = Class(ParticleForceGenerator)
  private
    other: PParticle;
    springConstant: Float;
    restLength: Float;
  public
    Constructor ParticleBungee(aother: PParticle; aspringConstant, arestLength: Float);
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  { ParticleBuoyancy }
  (**
   * A force generator that applies a buoyancy force for a plane of
   * liquid parrallel to XZ plane.
   *)
  ParticleBuoyancy = Class(ParticleForceGenerator)
  private
    maxDepth: float;
    volume: float;
    waterHeight: float;
    liquidDensity: float;
  public
    Constructor ParticleBuoyancy(amaxDepth, avolume, awaterHeight: FLoat; aliquidDensity: Float = 1000);
    Procedure updateForce(Var Particle: Particle; duration: float); override;
  End;

  TParticleForceRegistration = Record
    particle: PParticle;
    fg: ParticleForceGenerator;
  End;

  { ParticleForceRegistry }
  (**
   * Holds all the force generators and the particles they apply to.
   *)
  ParticleForceRegistry = Class
  private
    registrations: Array Of TParticleForceRegistration;
  public
    Constructor Create(); virtual;
    Destructor Destroy(); override;

    Procedure add(particle: PParticle; fg: ParticleForceGenerator);
    Procedure remove(particle: PParticle; fg: ParticleForceGenerator);
    Procedure Clear;
    Procedure updateForces(duration: real);
  End;

Implementation

{ ParticleGravity }

Constructor ParticleGravity.Create(aGravity: Vector3);
Begin
  gravity := aGravity;
End;

Procedure ParticleGravity.updateForce(Var Particle: Particle; duration: float);
Begin
  If Not Particle.hasFiniteMass Then exit;
  Particle.addForce(gravity * Particle.getMass());
End;

{ ParticleDrag }

Constructor ParticleDrag.Create(ak1, ak2: float);
Begin
  k1 := ak1;
  k2 := ak2;
End;

Procedure ParticleDrag.updateForce(Var Particle: Particle; duration: float);
Var
  force: Vector3;
  dragCoeff: float;
Begin
  particle.getVelocity(force);

  // Calculate the total drag coefficient
  dragCoeff := force.magnitude();
  dragCoeff := k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

  // Calculate the final force and apply it
  force.normalize();
  force := force * -dragCoeff;
  particle.addForce(force);
End;

{ ParticleAnchoredSpring }

Constructor ParticleAnchoredSpring.Create;
Begin

End;

Constructor ParticleAnchoredSpring.ParticleAnchoredSpring(aAnchor: PVector3;
  aSpringConstant, aRestLength: float);
Begin
  anchor := aAnchor;
  springConstant := aSpringConstant;
  restLength := aRestLength;
End;

Procedure ParticleAnchoredSpring.Init(aAnchor: PVector3; aSpringConstant,
  aRestLength: float);
Begin
  anchor := aAnchor;
  springConstant := aSpringConstant;
  restLength := aRestLength;
End;

Function ParticleAnchoredSpring.getAnchor: PVector3;
Begin
  result := anchor;
End;

Procedure ParticleAnchoredSpring.updateForce(Var Particle: Particle;
  duration: float);
Var
  force: Vector3;
  magnitude: float;
Begin
  // Calculate the vector of the spring

  particle.getPosition(force);
  force := force - anchor^;

  // Calculate the magnitude of the force
  magnitude := force.magnitude();
  magnitude := (restLength - magnitude) * springConstant;

  // Calculate the final force and apply it
  force.normalize();
  force := force * magnitude;
  particle.addForce(force);
End;

{ ParticleAnchoredBungee }

Procedure ParticleAnchoredBungee.updateForce(Var Particle: Particle;
  duration: float);
Var
  force: Vector3;
  magnitude: float;
Begin
  // Calculate the vector of the spring
  particle.getPosition(force);
  force := force - anchor^;

  // Calculate the magnitude of the force
  magnitude := force.magnitude();
  If (magnitude < restLength) Then exit;

  magnitude := magnitude - restLength;
  magnitude := magnitude * springConstant;

  // Calculate the final force and apply it
  force.normalize();
  force := force * -magnitude;
  particle.addForce(force);
End;

{ ParticleFakeSpring }

Constructor ParticleFakeSpring.ParticleFakeSpring(aAnchor: PVector3;
  aSpringConstant, aDamping: float);
Begin
  anchor := aAnchor;
  springConstant := aSpringConstant;
  damping := aDamping;
End;

Procedure ParticleFakeSpring.updateForce(Var Particle: Particle; duration: float
  );
Var
  accel, target, c, position: Vector3;
  gamma: float;

Begin
  // Check that we do not have infinite mass
  If (Not particle.hasFiniteMass()) Then exit;

  // Calculate the relative position of the particle to the anchor

  particle.getPosition(position);
  position := position - anchor^;

  // Calculate the constants and check they are in bounds.
  gamma := 0.5 * real_sqrt(4 * springConstant - damping * damping);
  If (gamma = 0.0) Then exit;
  c := position * (damping / (2.0 * gamma)) +
    particle.getVelocity() * (1.0 / gamma);

  // Calculate the target position
  target := position * real_cos(gamma * duration) +
    c * real_sin(gamma * duration);
  target := target * real_exp(-0.5 * duration * damping);

  // Calculate the resulting acceleration and therefore the force
  accel := (target - position) * (1.0 / (duration * duration)) -
    particle.getVelocity() * (1.0 / duration);
  particle.addForce(accel * particle.getMass());
End;

{ ParticleSpring }

Constructor ParticleSpring.ParticleSpring(aother: PParticle; aspringConstant,
  arestLength: Float);
Begin
  other := aother;
  springConstant := aspringConstant;
  restLength := arestLength;
End;

Procedure ParticleSpring.updateForce(Var Particle: Particle; duration: float);
Var
  force: Vector3;
  magnitude: float;
Begin
  // Calculate the vector of the spring

  particle.getPosition(force);
  force := force - other^.getPosition();

  // Calculate the magnitude of the force
  magnitude := force.magnitude();
  magnitude := real_abs(magnitude - restLength);
  magnitude := magnitude * springConstant;

  // Calculate the final force and apply it
  force.Normalize();
  force := force * -magnitude;
  particle.addForce(force);
End;

{ ParticleBungee }

Constructor ParticleBungee.ParticleBungee(aother: PParticle; aspringConstant,
  arestLength: Float);
Begin
  other := aother;
  springConstant := aspringConstant;
  restLength := arestLength;
End;

Procedure ParticleBungee.updateForce(Var Particle: Particle; duration: float);
Var
  force: Vector3;
  magnitude: Float;
Begin
  // Calculate the vector of the spring

  particle.getPosition(force);
  force := force - other^.getPosition();

  // Check if the bungee is compressed
  magnitude := force.magnitude();
  If (magnitude <= restLength) Then exit;

  // Calculate the magnitude of the force
  magnitude := springConstant * (restLength - magnitude);

  // Calculate the final force and apply it
  force.Normalize();
  force := force * -magnitude;
  particle.addForce(force);
End;

{ ParticleBuoyancy }

Constructor ParticleBuoyancy.ParticleBuoyancy(amaxDepth, avolume,
  awaterHeight: FLoat; aliquidDensity: Float);
Begin
  maxDepth := amaxDepth;
  volume := avolume;
  waterHeight := awaterHeight;
  liquidDensity := aliquidDensity;
End;

Procedure ParticleBuoyancy.updateForce(Var Particle: Particle; duration: float);
Var
  depth: float;
  force: Vector3;
Begin
  // Calculate the submersion depth
  depth := particle.getPosition().y;

  // Check if we're out of the water
  If (depth >= waterHeight + maxDepth) Then exit;
  force.create(0, 0, 0);

  // Check if we're at maximum depth
  If (depth <= waterHeight - maxDepth) Then Begin

    force.y := liquidDensity * volume;
    particle.addForce(force);
    exit;
  End;

  // Otherwise we are partly submerged
  force.y := liquidDensity * volume *
    (depth - maxDepth - waterHeight) / (2 * maxDepth);
  particle.addForce(force);
End;

{ ParticleForceRegistry }

Constructor ParticleForceRegistry.Create;
Begin
  registrations := Nil;
End;

Destructor ParticleForceRegistry.Destroy();
Begin
  Clear;
End;

Procedure ParticleForceRegistry.add(particle: PParticle;
  fg: ParticleForceGenerator);
Begin
  setlength(registrations, high(registrations) + 2);
  registrations[high(registrations)].particle := particle;
  registrations[high(registrations)].fg := fg;
End;

Procedure ParticleForceRegistry.remove(particle: PParticle;
  fg: ParticleForceGenerator);
Var
  i, j: Integer;
Begin
  For i := 0 To high(registrations) Do Begin
    If (registrations[i].particle = particle) And
      (registrations[i].fg = fg) Then Begin
      For j := i To high(registrations) - 1 Do Begin
        registrations[j] := registrations[j + 1];
      End;
      setlength(registrations, high(registrations));
    End;
  End;
End;

Procedure ParticleForceRegistry.Clear;
Begin
  setlength(registrations, 0);
End;

Procedure ParticleForceRegistry.updateForces(duration: real);
Var
  i: Integer;
Begin
  For i := 0 To high(registrations) Do Begin
    registrations[i].fg.updateForce(registrations[i].particle^, duration);
  End;
End;

End.

