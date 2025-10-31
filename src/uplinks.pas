Unit uplinks;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, upcontacts, uparticle, uprecision, ucore;

Type

  (**
   * Links connect two particles together, generating a contact if
   * they violate the constraints of their link. It is used as a
   * base class for cables and rods, and could be used as a base
   * class for springs with a limit to their extension..
   *)

  { ParticleLink }

  ParticleLink = Class(ParticleContactGenerator)
  private
  protected
    Function currentLength(): float; virtual;
  public
    particle: Array[0..1] Of PParticle;
  End;


  (**
   * Cables link a pair of particles, generating a contact if they
   * stray too far apart.
   *)

   { ParticleCable }

  ParticleCable = Class(ParticleLink)
  public
    maxLength: float;
    restitution: float;
    Function addContact(contact: PParticleContact; limit: integer): integer; override;
  End;
  PParticleCable = ^ParticleCable;

  (**
   * Rods link a pair of particles, generating a contact if they
   * stray too far apart or too close.
   *)

  { ParticleRod }

  ParticleRod = Class(ParticleLink)
  public
    length: float;
    Function addContact(contact: PParticleContact; limit: integer): integer; override;
  End;
  PParticleRod = ^ParticleRod;

  (**
   * Constraints are just like links, except they connect a particle to
   * an immovable anchor point.
   *)

  { ParticleConstraint }

  ParticleConstraint = Class(ParticleContactGenerator)
  public
    (**
     * Holds the particles connected by this constraint.
     *)
    particle: PParticle;

    (**
     * The point to which the particle is anchored.
     *)
    anchor: Vector3;
  protected
    (**
     * Returns the current length of the link.
     *)
    Function currentLength(): float; virtual;
  public

  End;

  (**
   * Cables link a particle to an anchor point, generating a contact if they
   * stray too far apart.
   *)

  { ParticleCableConstraint }

  ParticleCableConstraint = Class(ParticleConstraint)
  public
    (**
     * Holds the maximum length of the cable.
     *)
    maxLength: float;

    (**
     * Holds the restitution (bounciness) of the cable.
     *)
    restitution: float;

  public
    (**
     * Fills the given contact structure with the contact needed
     * to keep the cable from over-extending.
     *)
    Function addContact(contact: PParticleContact; limit: integer): integer; override;
  End;

  PParticleCableConstraint = ^ParticleCableConstraint;

  { ParticleRodConstraint }

  ParticleRodConstraint = Class(ParticleConstraint)
  public
    (**
     * Holds the length of the rod.
     *)
    length: float;
  public
    (**
     * Fills the given contact structure with the contact needed
     * to keep the rod from extending or compressing.
     *)
    Function addContact(contact: PParticleContact; limit: integer): integer; override;
  End;

Implementation

{ ParticleLink }

Function ParticleLink.currentLength: float;
Var
  relativePos: Vector3;
Begin
  relativePos := particle[0]^.getPosition() -
    particle[1]^.getPosition();
  result := relativePos.magnitude();
End;

{ ParticleCable }

Function ParticleCable.addContact(contact: PParticleContact; limit: integer): integer;
Var
  length: float;
  normal: Vector3;
Begin
  result := 0;
  // Find the length of the cable
  length := currentLength();

  // Check if we're over-extended
  If (length < maxLength) Then Begin
    exit;
  End;

  // Otherwise return the contact
  contact[0].particle[0] := particle[0];
  contact[0].particle[1] := particle[1];

  // Calculate the normal
  normal := particle[1]^.getPosition() - particle[0]^.getPosition();
  normal.Normalize();
  contact[0].contactNormal := normal;

  contact[0].penetration := length - maxLength;
  contact[0].restitution := restitution;

  result := 1;
End;

{ ParticleRod }

Function ParticleRod.addContact(contact: PParticleContact; limit: integer): integer;
Var
  currentLen: float;
  normal: Vector3;
Begin
  result := 0;
  // Find the length of the rod
  currentLen := currentLength();

  // Check if we're over-extended
  If (currentLen = length) Then Begin
    exit;
  End;

  // Otherwise return the contact
  contact[0].particle[0] := particle[0];
  contact[0].particle[1] := particle[1];

  // Calculate the normal
  normal := particle[1]^.getPosition() - particle[0]^.getPosition();
  normal.Normalize();

  // The contact normal depends on whether we're extending or compressing
  If (currentLen > length) Then Begin
    contact[0].contactNormal := normal;
    contact[0].penetration := currentLen - length;
  End
  Else Begin
    contact[0].contactNormal := normal * -1;
    contact[0].penetration := length - currentLen;
  End;

  // Always use zero restitution (no bounciness)
  contact[0].restitution := 0;

  result := 1;
End;

{ ParticleConstraint }

Function ParticleConstraint.currentLength: float;
Var
  relativePos: Vector3;
Begin
  relativePos := particle^.getPosition() - anchor;
  result := relativePos.magnitude();
End;

{ ParticleCableConstraint }

Function ParticleCableConstraint.addContact(contact: PParticleContact;
  limit: integer): integer;
Var
  length: Float;
  normal: Vector3;
Begin
  result := 0;
  // Find the length of the cable
  length := currentLength();

  // Check if we're over-extended
  If (length < maxLength) Then Begin
    exit;
  End;

  // Otherwise return the contact
  contact^.particle[0] := particle;
  contact^.particle[1] := Nil;

  // Calculate the normal
  normal := anchor - particle^.getPosition();
  normal.Normalize();
  contact^.contactNormal := normal;

  contact^.penetration := length - maxLength;
  contact^.restitution := restitution;

  result := 1;
End;

{ ParticleRodConstraint }

Function ParticleRodConstraint.addContact(contact: PParticleContact;
  limit: integer): integer;
Var
  currentLen: Float;
  normal: Vector3;
Begin
  result := 0;
  // Find the length of the rod
  currentLen := currentLength();

  // Check if we're over-extended
  If (currentLen = length) Then Begin
    exit;
  End;

  // Otherwise return the contact
  contact^.particle[0] := particle;
  contact^.particle[1] := Nil;

  // Calculate the normal
  normal := anchor - particle^.getPosition();
  normal.Normalize();

  // The contact normal depends on whether we're extending or compressing
  If (currentLen > length) Then Begin
    contact^.contactNormal := normal;
    contact^.penetration := currentLen - length;
  End
  Else Begin
    contact^.contactNormal := normal * -1;
    contact^.penetration := length - currentLen;
  End;
  // Always use zero restitution (no bounciness)
  contact^.restitution := 0;
  result := 1;
End;

End.

