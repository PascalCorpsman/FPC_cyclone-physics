Unit uplinks;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, upcontacts, uparticle, uprecision, ucore;

Type

  { ParticleLink }

  ParticleLink = Class(ParticleContactGenerator)
  private
  protected
    Function currentLength(): float;
  public
    particle: Array[0..1] Of PParticle;
  End;

  { ParticleCable }

  ParticleCable = Class(ParticleLink)
  public
    maxLength: float;
    restitution: float;
    Function addContact(Var contact: Array Of ParticleContact; limit: integer): integer; override;
  End;

  { ParticleRod }

  ParticleRod = Class(ParticleLink)
  public
    length: float;
    Function addContact(Var contact: Array Of ParticleContact; limit: integer): integer; override;
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

Function ParticleCable.addContact(Var contact: Array Of ParticleContact;
  limit: integer): integer;
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

Function ParticleRod.addContact(Var contact: Array Of ParticleContact;
  limit: integer): integer;
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

End.

