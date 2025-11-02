(******************************************************************************)
(*                                                                            *)
(* Author      : Uwe Schächterle (Corpsman)                                   *)
(*                                                                            *)
(* This file is part of FPC_cyclone-physics-demos                             *)
(*                                                                            *)
(*  See the file license.md, located under:                                   *)
(*  https://github.com/PascalCorpsman/Software_Licenses/blob/main/license.md  *)
(*  for details about the license.                                            *)
(*                                                                            *)
(*               It is not allowed to change or remove this text from any     *)
(*               source file of the project.                                  *)
(*                                                                            *)
(******************************************************************************)
Unit ublob;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, upcontacts, upfgen, utiming;

Const
  BLOB_COUNT = 5;
  PLATFORM_COUNT = 10;
  BLOB_RADIUS = 0.4;

Type

  (**
   * Platforms are two dimensional: lines on which the
   * particles can rest. Platforms are also contact generators for the physics.
   *)

  { Platform }

  Platform = Class(ParticleContactGenerator)
  public
    _start: Vector3;
    _end: Vector3;

    (**
     * Holds a pointer to the particles we're checking for collisions with.
     *)
    particles: PParticle;

    Function addContact(contact: PParticleContact; limit: integer): integer; override;
  End;

  (**
   * A force generator for proximal attraction.
   *)

  { BlobForceGenerator }

  BlobForceGenerator = Class(ParticleForceGenerator)
    (**
     * Holds a pointer to the particles we might be attracting.
     *)
    particles: PParticle;

    (**
     * The maximum force used to push the particles apart.
     *)
    maxReplusion: Float;

    (**
     * The maximum force used to pull particles together.
     *)
    maxAttraction: Float;

    (**
     * The separation between particles where there is no force.
     *)
    minNaturalDistance, maxNaturalDistance: Float;

    (**
     * The force with which to float the head particle, if it is
     * joined to others.
     *)
    floatHead: Float;

    (**
     * The maximum number of particles in the blob before the head
     * is floated at maximum force.
     *)
    maxFloat: unsigned;

    (**
     * The separation between particles after which they 'break' apart and
     * there is no force.
     *)
    maxDistance: Float;

    Procedure updateForce(Var aParticle: Particle; duration: float); override;
  End;

  { BlobDemo }

  BlobDemo = Class(MassAggregateApplication)
  private
    blobs: Array Of Particle;

    platforms: Array Of Platform;

    _blobForceGenerator: BlobForceGenerator;

    (* The control for the x-axis. *)
    xAxis: float;

    (* The control for the y-axis. *)
    yAxis: float;
    Procedure Reset();
  public
    Constructor Create(); virtual; reintroduce;
    Destructor Destroy(); override;

    (** Returns the window title for the demo. *)
    Function GetTitle: String; override;

    (** Display the particles. *)
    Procedure Display; override;

    (** Update the particle positions. *)
    Procedure Update; override;

    (** Handle a key press. *)
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := BlobDemo.Create;
End;

{ Platform }

Function Platform.addContact(contact: PParticleContact; limit: integer
  ): integer;
Const
  restitution = 0.0;
Var
  used: unsigned;
  i: Integer;
  closestPoint, toParticle, lineDirection: Vector3;
  distanceToPlatform, projected, platformSqLength: Float;
Begin
  used := 0;

  For i := 0 To BLOB_COUNT - 1 Do Begin
    If (used >= limit) Then break;

    // Check for penetration
    toParticle := particles[i].getPosition() - _start;
    lineDirection := _end - _start;
    projected := toParticle * lineDirection;
    platformSqLength := lineDirection.squareMagnitude();
    If (projected <= 0) Then Begin

      // The blob is nearest to the start point
      If (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS) Then Begin

        // We have a collision
        contact^.contactNormal := toParticle._unit();
        contact^.contactNormal.z := 0;
        contact^.restitution := restitution;
        contact^.particle[0] := particles + i;
        contact^.particle[1] := Nil;
        contact^.penetration := BLOB_RADIUS - toParticle.magnitude();
        inc(used);
        inc(contact);
      End;

    End
    Else If (projected >= platformSqLength) Then Begin

      // The blob is nearest to the end point
      toParticle := particles[i].getPosition() - _end;
      If (toParticle.squareMagnitude() < BLOB_RADIUS * BLOB_RADIUS) Then Begin

        // We have a collision
        contact^.contactNormal := toParticle._unit();
        contact^.contactNormal.z := 0;
        contact^.restitution := restitution;
        contact^.particle[0] := particles + i;
        contact^.particle[1] := Nil;
        contact^.penetration := BLOB_RADIUS - toParticle.magnitude();
        inc(used);
        inc(contact);
      End;
    End
    Else Begin
      // the blob is nearest to the middle.
      distanceToPlatform :=
        toParticle.squareMagnitude() -
        projected * projected / platformSqLength;
      If (distanceToPlatform < BLOB_RADIUS * BLOB_RADIUS) Then Begin

        // We have a collision
        closestPoint := _start + lineDirection * (projected / platformSqLength);

        contact^.contactNormal := (particles[i].getPosition() - closestPoint)._unit();
        contact^.contactNormal.z := 0;
        contact^.restitution := restitution;
        contact^.particle[0] := particles + i;
        contact^.particle[1] := Nil;
        contact^.penetration := BLOB_RADIUS - real_sqrt(distanceToPlatform);
        inc(used);
        inc(contact);
      End;
    End;
  End;
  result := used;
End;

{ BlobForceGenerator }

Procedure BlobForceGenerator.updateForce(Var aParticle: Particle; duration: float
  );
Var
  joinCount: unsigned;
  i: Integer;
  separation: Vector3;
  force, distance: Float;
Begin
  joinCount := 0;
  For i := 0 To BLOB_COUNT - 1 Do Begin

    // Don't attract yourself
    If (@particles[i] = @aparticle) Then continue;

    // Work out the separation distance
    separation :=
      particles[i].getPosition() - aparticle.getPosition();
    separation.z := 0.0;
    distance := separation.magnitude();

    If (distance < minNaturalDistance) Then Begin

      // Use a repulsion force.
      distance := 1.0 - distance / minNaturalDistance;
      aparticle.addForce(
        separation._unit() * (1.0 - distance) * maxReplusion * -1.0
        );
      inc(joinCount);

    End
    Else If (distance > maxNaturalDistance) And (distance < maxDistance) Then Begin

      // Use an attraction force.
      distance :=
        (distance - maxNaturalDistance) /
        (maxDistance - maxNaturalDistance);
      aparticle.addForce(
        separation._unit() * distance * maxAttraction
        );
      inc(joinCount);
    End;
  End;

  // If the particle is the head, and we've got a join count, then float it.
  If (@aparticle = @particles[0]) And (joinCount > 0) And (maxFloat > 0) Then Begin
    force := (joinCount / maxFloat) * floatHead;
    If (force > floatHead) Then force := floatHead;
    aparticle.addForce(V3(0, force, 0));
  End;
End;

Constructor BlobDemo.Create;
Var
  i: Integer;
  p: ^Platform;
  fraction: Float;
  delta: Vector3;
  me: unsigned;
Begin
  Inherited create(PLATFORM_COUNT + BLOB_COUNT, PLATFORM_COUNT);
  xAxis := 0;
  yAxis := 0;
  // Create the blob storage
  setlength(blobs, BLOB_COUNT);

  // Create the force generator
  _blobForceGenerator := blobForceGenerator.create;
  _blobForceGenerator.particles := @blobs[0];
  _blobForceGenerator.maxAttraction := 20.0;
  _blobForceGenerator.maxReplusion := 10.0;
  _blobForceGenerator.minNaturalDistance := BLOB_RADIUS * 0.75;
  _blobForceGenerator.maxNaturalDistance := BLOB_RADIUS * 1.5;
  _blobForceGenerator.maxDistance := BLOB_RADIUS * 2.5;
  _blobForceGenerator.maxFloat := 2;
  _blobForceGenerator.floatHead := 8.0;

  // Create the platforms
  setlength(platforms, PLATFORM_COUNT);
  For i := 0 To high(platforms) Do Begin
    platforms[i] := Platform.Create;


    platforms[i]._start := V3(
      (i Mod 2) * 10.0 - 5.0,
      (i) * 4.0 + (ifthen(i Mod 2 <> 0, 0.0, 2.0)),
      0);
    platforms[i]._start.x := platforms[i]._start.x + Random.randomBinomial(2.0);
    platforms[i]._start.y := platforms[i]._start.y + Random.randomBinomial(2.0);

    platforms[i]._end := V3(
      (i Mod 2) * 10.0 + 5.0,
      (i) * 4.0 + (ifthen(i Mod 2 <> 0, 2.0, 0.0)),
      0);
    platforms[i]._end.x := platforms[i]._end.x + Random.randomBinomial(2.0);
    platforms[i]._end.y := platforms[i]._end.y + Random.randomBinomial(2.0);

    // Make sure the platform knows which particles it
    // should collide with.
    platforms[i].particles := @blobs[0];
    world.getContactGenerators().push_back(platforms[i]);
  End;

  // Create the blobs.
  p := @platforms[(PLATFORM_COUNT - 2)];
  fraction := 1.0 / BLOB_COUNT;
  delta := p^._end - p^._start;
  For i := 0 To BLOB_COUNT - 1 Do Begin
    me := (i + BLOB_COUNT Div 2) Mod BLOB_COUNT;
    blobs[i].setPosition(
      p^._start + delta * ((me) * 0.8 * fraction + 0.1) +
      V3(0, 1.0 + Random.randomReal(), 0));

    blobs[i].setVelocity(0, 0, 0);
    blobs[i].setDamping(0.2);
    blobs[i].setAcceleration(GRAVITY * 0.4);
    blobs[i].setMass(1.0);
    blobs[i].clearAccumulator();

    world.getParticles().push_back(@blobs[i]);
    world.getForceRegistry().add(@blobs[i], _blobForceGenerator);
  End;
End;

Destructor BlobDemo.Destroy;
Var
  i: Integer;
Begin
  For i := 0 To high(platforms) Do
    platforms[i].Free;
  Inherited Destroy();
End;

Procedure BlobDemo.Reset;
Var
  p: ^Platform;
  fraction: float;
  delta: Vector3;
  me: unsigned;
  i: Integer;
Begin
  p := @platforms[(PLATFORM_COUNT - 2)];
  fraction := 1.0 / BLOB_COUNT;
  delta := p^._end - p^._start;
  For i := 0 To BLOB_COUNT - 1 Do Begin
    me := (i + BLOB_COUNT Div 2) Mod BLOB_COUNT;
    blobs[i].setPosition(
      p^._start + delta * (me * 0.8 * fraction + 0.1) +
      V3(0, 1.0 + random.randomReal(), 0));
    blobs[i].setVelocity(0, 0, 0);
    blobs[i].clearAccumulator();
  End;
End;

Function BlobDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Blob Demo';
End;

Procedure BlobDemo.Display;
Var
  v, p, p0, p1, pos: Vector3;
  i: Integer;

Begin
  pos := blobs[0].getPosition();

  // Clear the view port and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(pos.x, pos.y, 6.0, pos.x, pos.y, 0.0, 0.0, 1.0, 0.0);

  glColor3f(0, 0, 0);

  glBegin(GL_LINES);
  glColor3f(0, 0, 1);
  For i := 0 To high(platforms) Do Begin
    p0 := platforms[i]._start;
    p1 := platforms[i]._end;
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
  End;
  glEnd();

  glColor3f(1, 0, 0);
  For i := 0 To BLOB_COUNT - 1 Do Begin

    p := blobs[i].getPosition();
    glPushMatrix();
    glTranslatef(p.x, p.y, p.z);
    glutSolidSphere(BLOB_RADIUS, 12, 12);
    glPopMatrix();
  End;

  p := blobs[0].getPosition();
  v := blobs[0].getVelocity() * 0.05;
  v.trim(BLOB_RADIUS * 0.5);
  p := p + v;
  glPushMatrix();
  glTranslatef(p.x - BLOB_RADIUS * 0.2, p.y, BLOB_RADIUS);
  glColor3f(1, 1, 1);
  glutSolidSphere(BLOB_RADIUS * 0.2, 8, 8);
  glTranslatef(0, 0, BLOB_RADIUS * 0.2);
  glColor3f(0, 0, 0);
  glutSolidSphere(BLOB_RADIUS * 0.1, 8, 8);
  glTranslatef(BLOB_RADIUS * 0.4, 0, -BLOB_RADIUS * 0.2);
  glColor3f(1, 1, 1);
  glutSolidSphere(BLOB_RADIUS * 0.2, 8, 8);
  glTranslatef(0, 0, BLOB_RADIUS * 0.2);
  glColor3f(0, 0, 0);
  glutSolidSphere(BLOB_RADIUS * 0.1, 8, 8);
  glPopMatrix();
End;

Procedure BlobDemo.Update;
Var
  duration: Float;
  position: Vector3;
  i: Integer;
Begin
  // Clear accumulators
  world.startFrame();

  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  // Recenter the axes
  xAxis := xAxis * real_pow(0.1, duration);
  yAxis := yAxis * real_pow(0.1, duration);

  // Move the controlled blob
  blobs[0].addForce(V3(xAxis, yAxis, 0) * 10.0);

  // Run the simulation
  world.runPhysics(duration);

  // Bring all the particles back to 2d
  For i := 0 To BLOB_COUNT - 1 Do Begin
    blobs[i].getPosition(position);
    position.z := 0.0;
    blobs[i].setPosition(position);
  End;

  Inherited update();
End;

Procedure BlobDemo.Key(akey: Char);
Begin
  Case akey Of
    's', 'S': Begin
        yAxis := -1.0;
      End;
    'w', 'W': Begin
        yAxis := 1.0;
      End;
    'a', 'A': Begin
        xAxis := -1.0;
      End;
    'd', 'D': Begin
        xAxis := 1.0;
      End;
    'r', 'R': Begin
        Reset;
      End;
  End;
End;

End.

