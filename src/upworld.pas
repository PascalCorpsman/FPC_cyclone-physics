Unit upworld;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision, uparticle, upcontacts;

Type

  TParticles = specialize TArray < PVector3 > ;
  TContactGenerators = specialize TArray < ParticleContactGenerator > ;

  { ParticleWorld }

  ParticleWorld = Class
  private
  protected
    Particles: TParticles;
    contactGenerators: TContactGenerators;
  public
    Constructor Create(maxContacts: integer; iterations: integer = 0);
    Function getParticles(): TParticles;
    Function getContactGenerators(): TContactGenerators;
  End;

  { GroundContacts }

  GroundContacts = Class(ParticleContactGenerator)
    Particles: Tparticles;
    Procedure init(aParticles: Tparticles);
  End;

Implementation

{ ParticleWorld }

Constructor ParticleWorld.Create(maxContacts: integer; iterations: integer);
Begin
  Particles := TParticles.Create;
  contactGenerators := TcontactGenerators.Create();
End;

Function ParticleWorld.getParticles: TParticles;
Begin
  result := Particles;
End;

Function ParticleWorld.getContactGenerators(): TContactGenerators;
Begin
  result := contactGenerators;
End;

{ GroundContacts }

Procedure GroundContacts.init(aParticles: Tparticles);
Begin
  Particles := aParticles;
End;

End.

