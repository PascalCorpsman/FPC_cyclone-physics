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
Unit upcontacts;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision, uparticle;

Type

  ParticleContactResolver = Class;

  { ParticleContact }

  ParticleContact = Object
  public
    Particle: Array[0..1] Of PParticle;
    restitution: float;
    ContactNormal: Vector3;
    penetration: Float;
    particleMovement: Array[0..1] Of Vector3;
  private
    Procedure resolveVelocity(duration: float);
    Procedure resolveInterpenetration(duration: float);
  protected
    Procedure Resolve(duration: Float);
    Function CalculateSeparatingVelocity(): float;
  End;

  { ParticleContactGenerator }

  ParticleContactGenerator = Class
    Function addContact(Var contact: Array Of ParticleContact; limit: integer): integer; virtual;
  End;

  PParticleContactGenerator = ^ParticleContactGenerator;

  { ParticleContactResolver }

  ParticleContactResolver = Class
  private
    iterations: integer;
    iterationsUsed: integer;
  public
    Constructor Create(aiterations: integer);
    Procedure setIterations(aiterations: integer);
    Procedure resolveContacts(Const contactArray: Array Of ParticleContact; numContacts: integer; duration: float);
  End;

Implementation

{ ParticleContact }

Procedure ParticleContact.resolveVelocity(duration: float);
Var
  impulse, totalInverseMass, deltaVelocity, accCausedSepVelocity, separatingVelocity, newSepVelocity: float;
  impulsePerIMass, accCausedVelocity: Vector3;
Begin
  // Find the velocity in the direction of the contact
  separatingVelocity := calculateSeparatingVelocity();

  // Check if it needs to be resolved
  If (separatingVelocity > 0) Then Begin
    // The contact is either separating, or stationary - there's
    // no impulse required.
    exit;
  End;

  // Calculate the new separating velocity
  newSepVelocity := -separatingVelocity * restitution;

  // Check the velocity build-up due to acceleration only
  accCausedVelocity := particle[0]^.getAcceleration();
  If assigned(particle[1]) Then Begin
    accCausedVelocity := accCausedVelocity - particle[1]^.getAcceleration();
  End;
  accCausedSepVelocity := accCausedVelocity * contactNormal * duration;

  // If we've got a closing velocity due to acceleration build-up,
  // remove it from the new separating velocity
  If (accCausedSepVelocity < 0) Then Begin

    newSepVelocity := newSepVelocity + restitution * accCausedSepVelocity;

    // Make sure we haven't removed more than was
    // there to remove.
    If (newSepVelocity < 0) Then newSepVelocity := 0;
  End;

  deltaVelocity := newSepVelocity - separatingVelocity;

  // We apply the change in velocity to each object in proportion to
  // their inverse mass (i.e. those with lower inverse mass [higher
  // actual mass] get less change in velocity)..
  totalInverseMass := particle[0]^.getInverseMass();
  If assigned(particle[1]) Then Begin
    totalInverseMass := totalInverseMass + particle[1]^.getInverseMass();
  End;

  // If all particles have infinite mass, then impulses have no effect
  If (totalInverseMass <= 0) Then exit;

  // Calculate the impulse to apply
  impulse := deltaVelocity / totalInverseMass;

  // Find the amount of impulse per unit of inverse mass
  impulsePerIMass := contactNormal * impulse;

  // Apply impulses: they are applied in the direction of the contact,
  // and are proportional to the inverse mass.
  particle[0]^.setVelocity(particle[0]^.getVelocity() +
    impulsePerIMass * particle[0]^.getInverseMass()
    );
  If assigned(particle[1]) Then Begin
    // Particle 1 goes in the opposite direction
    particle[1]^.setVelocity(particle[1]^.getVelocity() +
      impulsePerIMass * -particle[1]^.getInverseMass()
      );
  End;
End;

Procedure ParticleContact.resolveInterpenetration(duration: float);
Var
  totalInverseMass: float;
  movePerIMass: Vector3;
Begin
  // If we don't have any penetration, skip this step.
  If (penetration <= 0) Then exit;

  // The movement of each object is based on their inverse mass, so
  // total that.
  totalInverseMass := particle[0]^.getInverseMass();
  If assigned(particle[1]) Then Begin
    totalInverseMass := totalInverseMass + particle[1]^.getInverseMass();
  End;

  // If all particles have infinite mass, then we do nothing
  If (totalInverseMass <= 0) Then exit;

  // Find the amount of penetration resolution per unit of inverse mass
  movePerIMass := contactNormal * (penetration / totalInverseMass);

  // Calculate the the movement amounts
  particleMovement[0] := movePerIMass * particle[0]^.getInverseMass();
  If assigned(particle[1]) Then Begin
    particleMovement[1] := movePerIMass * -particle[1]^.getInverseMass();
  End
  Else Begin
    particleMovement[1].clear();
  End;

  // Apply the penetration resolution
  particle[0]^.setPosition(particle[0]^.getPosition() + particleMovement[0]);
  If assigned(particle[1]) Then Begin
    particle[1]^.setPosition(particle[1]^.getPosition() + particleMovement[1]);
  End;
End;

Procedure ParticleContact.Resolve(duration: Float);
Begin
  resolveVelocity(duration);
  resolveInterpenetration(duration);
End;

Function ParticleContact.CalculateSeparatingVelocity: float;
Var
  relativeVelocity: Vector3;
Begin
  relativeVelocity := particle[0]^.getVelocity();
  If assigned(particle[1]) Then Begin
    relativeVelocity := relativeVelocity - particle[1]^.getVelocity();
  End;
  result := relativeVelocity * contactNormal;
End;

{ ParticleContactGenerator }

Function ParticleContactGenerator.addContact(
  Var contact: Array Of ParticleContact; limit: integer): integer;
Begin
  result := 0;
End;

{ ParticleContactResolver }

Constructor ParticleContactResolver.Create(aiterations: integer);
Begin
  iterations := aiterations;
End;

Procedure ParticleContactResolver.setIterations(aiterations: integer);
Begin
  iterations := aiterations;
End;

Procedure ParticleContactResolver.resolveContacts(
  Const contactArray: Array Of ParticleContact; numContacts: integer;
  duration: float);
Begin

End;

End.

