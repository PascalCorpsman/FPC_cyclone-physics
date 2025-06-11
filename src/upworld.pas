Unit upworld;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision, uparticle, upcontacts, upfgen;

Type

  TParticles = specialize TArray < PParticle > ;
  TContactGenerators = specialize TArray < ParticleContactGenerator > ;

  { ParticleWorld }

  ParticleWorld = Class
  private
  protected
    Particles: TParticles;
    contactGenerators: TContactGenerators;
    calculateIterations: Boolean;
    resolver: ParticleContactResolver;
    contacts: Array Of ParticleContact;
    maxContacts: integer;
    Procedure integrate(diration: float);
    Function generateContacts(): integer;
  public
    registry: ParticleForceRegistry;
    Constructor Create(amaxContacts: integer; iterations: integer = 0);
    Destructor Destroy; override;
    Function getParticles(): TParticles;
    Function getContactGenerators(): TContactGenerators;

    Procedure startFrame();
    Procedure runPhysics(duration: Float);

  End;

  { GroundContacts }

  GroundContacts = Class(ParticleContactGenerator)
    Particles: Tparticles;
    Procedure init(aParticles: Tparticles);
  End;

Implementation

{ ParticleWorld }

Procedure ParticleWorld.integrate(diration: float);
Var
  iterator: integer;
Begin
  For iterator := 0 To Particles.Count - 1 Do Begin
    Particles[iterator]^.Integrate(diration);
  End;
End;

Function ParticleWorld.generateContacts(): integer;
Var
  limit: integer;
Begin
  limit := maxContacts;
  //    ParticleContact *nextContact = contacts;
  //
  //    for (ContactGenerators::iterator g = contactGenerators.begin();
  //        g != contactGenerators.end();
  //        g++)
  //    {
  //        unsigned used =(*g)->addContact(nextContact, limit);
  //        limit -= used;
  //        nextContact += used;
  //
  //        // We've run out of contacts to fill. This means we're missing
  //        // contacts.
  //        if (limit <= 0) break;
  //    }
  //
  //    // Return the number of contacts used.
  result := maxContacts - limit;
End;

Constructor ParticleWorld.Create(amaxContacts: integer; iterations: integer);
Begin
  Particles := TParticles.Create;
  contactGenerators := TcontactGenerators.Create();
  registry := ParticleForceRegistry.Create();
  resolver := ParticleContactResolver.create(iterations);
  maxContacts := amaxContacts;
  contacts := Nil;
  setlength(contacts, amaxContacts);
  calculateIterations := (iterations = 0);
End;

Destructor ParticleWorld.Destroy;
Begin
  resolver.free;
  Particles.Free;
  contactGenerators.Free;
  registry.Free;
End;

Function ParticleWorld.getParticles: TParticles;
Begin
  result := Particles;
End;

Function ParticleWorld.getContactGenerators: TContactGenerators;
Begin
  result := contactGenerators;
End;

Procedure ParticleWorld.startFrame;
Var
  iterator: integer;
  p: PParticle;
Begin
  For iterator := 0 To Particles.Count - 1 Do Begin
    p := Particles[iterator];
    // Remove all forces from the accumulator
    p^.clearAccumulator();
  End;
End;

Procedure ParticleWorld.runPhysics(duration: Float);
Var
  usedContacts: integer;
Begin
  // First apply the force generators

  registry.updateForces(duration);

  // Then integrate the objects
  integrate(duration);

  // Generate contacts
  usedContacts := generateContacts();

  // And process them
  If (usedContacts <> 0) Then Begin

    If (calculateIterations) Then resolver.setIterations(usedContacts * 2);
    resolver.resolveContacts(contacts, usedContacts, duration);
  End;
End;

{ GroundContacts }

Procedure GroundContacts.init(aParticles: Tparticles);
Begin
  Particles := aParticles;
End;

End.

