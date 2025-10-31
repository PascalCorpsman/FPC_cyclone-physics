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
    Function addContact(contact: PParticleContact; limit: integer): integer; override;
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
  nextContact, i: integer;
  g: ParticleContactGenerator;
  used: Unsigned;
Begin
  limit := maxContacts;
  nextContact := 0;
  For i := 0 To contactGenerators.Count - 1 Do Begin
    g := contactGenerators.Element[i];
    used := g.addContact(@contacts[nextContact], limit);
    limit := limit - used;
    nextContact := nextContact + used;

    // We've run out of contacts to fill. This means we're missing
    // contacts.
    If (limit <= 0) Then break;
  End;
  // Return the number of contacts used.
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

Function GroundContacts.addContact(contact: PParticleContact; limit: integer
  ): integer;
Var
  count: unsigned;
  y: float;
  i: Integer;
Begin
  count := 0;
  For i := 0 To Particles.Count - 1 Do Begin
    y := Particles.Element[i]^.getPosition().y;
    If (y < 0.0) Then Begin
      contact^.contactNormal := UP;
      contact^.particle[0] := Particles.Element[i];
      contact^.particle[1] := Nil;
      contact^.penetration := -y;
      contact^.restitution := 0.2;
      contact := contact + 1;
      count := count + 1;
    End;
    If (count >= limit) Then Begin
      result := count;
      exit;
    End;
  End;
  result := count;
End;

End.

