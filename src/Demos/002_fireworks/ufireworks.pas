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
Unit ufireworks;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uApp, ucore, uprecision, uparticle, urandom, utiming;

Const
  ruleCount = 9;
  maxFireworks = 1024;

Type

  { TFirework }

  TFirework = Object(Particle)
  public
    _type: integer;
    age: float;
    Function Update(duration: FLoat): Boolean;
  End;

  PFirework = ^TFirework;

  { TPayload }

  TPayload = Object
    _type: integer;
    count: integer;
    Procedure _Set(atype, aCount: integer);
  End;

  { FireworkRule }

  FireworkRule = Object
    _type: integer;
    minAge: Float;
    maxAge: Float;
    minVelocity: Vector3;
    maxVelocity: Vector3;
    damping: Float;
    Payload: TPayload;
    payloadCount: integer;
    payloads: Array Of TPayload;
  public
    Procedure Init(apayloadCount: integer);
    Procedure SetParameters(a_type: integer;
      aminAge, amaxAge: float;
      aminVelocity, amaxVelocity: Vector3;
      adamping: float);
    Constructor Create; overload;
    Procedure Create(Var firework: TFirework; Parent: PFirework);
  End;

  { FireworksDemo }

  FireworksDemo = Class(Application)
  private
    fireworks: Array[0..maxFireworks - 1] Of TFirework;
    nextFirework: integer;
    rules: Array[0..ruleCount - 1] Of FireworkRule;
    Procedure Create(_type: integer; parent: PFirework); overload;
    Procedure Create(_type, number: integer; parent: PFirework); overload;
    Procedure initFireworkRules;
  public
    Constructor Create(); override;

    Procedure initGraphics(); override;
    Function GetTitle: String; override;
    Procedure Update; override;
    Procedure Display; override;
    Procedure Key(akey: Char); override;
  End;

Function getApplication(): Application;

Implementation

Uses dglOpenGL;

Function getApplication(): Application;
Begin
  result := FireworksDemo.Create();
End;

{ TFirework }

Function TFirework.Update(duration: FLoat): Boolean;
Begin
  Integrate(duration);
  age := age - duration;
  result := (age < 0) Or (position.y < 0);
End;

{ TPayload }

Procedure TPayload._Set(atype, aCount: integer);
Begin
  _type := atype;
  count := aCount;
End;

{ FireworkRule }

Procedure FireworkRule.Init(apayloadCount: integer);
Begin
  payloadCount := apayloadCount;
  setlength(payloads, apayloadCount);
End;

Procedure FireworkRule.SetParameters(a_type: integer; aminAge, amaxAge: float;
  aminVelocity, amaxVelocity: Vector3; adamping: float);
Begin
  _type := a_type;
  minAge := aminAge;
  maxAge := amaxAge;
  minVelocity := aminVelocity;
  maxVelocity := amaxVelocity;
  damping := adamping;
End;

Constructor FireworkRule.Create;
Begin
  payloadCount := 0;
  payloads := Nil;
End;

Procedure FireworkRule.Create(Var firework: TFirework; Parent: PFirework);
Var
  start, vel: Vector3;
  x: integer;
Begin
  firework._type := _type;
  firework.age := random.randomReal(minAge, maxAge);

  vel.create();
  If assigned(parent) Then Begin
    // The position and velocity are based on the parent.
    firework.setPosition(parent^.getPosition());
    vel := vel + parent^.getVelocity();
  End
  Else Begin
    start.create();
    x := random.randomInt(3) - 1;
    start.x := 5.0 * x;
    firework.setPosition(start);
  End;

  vel := vel + random.randomVector(minVelocity, maxVelocity);
  firework.setVelocity(vel);

  // We use a mass of one in all cases (no point having fireworks
  // with different masses, since they are only under the influence
  // of gravity).
  firework.setMass(1);
  firework.setDamping(damping);
  firework.setAcceleration(GRAVITY);
  firework.clearAccumulator();
End;

{ FireworksDemo }

Procedure FireworksDemo.Create(_type: integer; parent: PFirework);
Var
  rule: Integer;
Begin
  // Get the rule needed to create this firework
  rule := (_type - 1);

  // Create the firework
  rules[rule].create(fireworks[nextFirework], parent);

  // Increment the index for the next firework
  nextFirework := (nextFirework + 1) Mod maxFireworks;
End;

Procedure FireworksDemo.Create(_type, number: integer; parent: PFirework);
Var
  i: Integer;
Begin
  For i := 0 To number - 1 Do Begin
    create(_type, parent);
  End;
End;

Procedure FireworksDemo.initFireworkRules;
Begin
  // Go through the firework types and create their rules.
  rules[0].init(2);
  rules[0].setParameters(
    1, // type
    0.5, 1.4, // age range
    V3(-5, 25, -5), // min velocity
    V3(5, 28, 5.0), // max velocity
    0.1 // damping
    );
  rules[0].payloads[0]._Set(3, 5);
  rules[0].payloads[1]._Set(5, 5);

  rules[1].init(1);
  rules[1].setParameters(
    2, // type
    0.5, 1.0, // age range
    V3(-5, 10, -5), // min velocity
    V3(5, 20, 5), // max velocity
    0.8 // damping
    );
  rules[1].payloads[0]._set(4, 2);

  rules[2].init(0);
  rules[2].setParameters(
    3, // type
    0.5, 1.5, // age range
    V3(-5, -5, -5), // min velocity
    V3(5, 5, 5), // max velocity
    0.1 // damping
    );

  rules[3].init(0);
  rules[3].setParameters(
    4, // type
    0.25, 0.5, // age range
    V3(-20, 5, -5), // min velocity
    V3(20, 5, 5), // max velocity
    0.2 // damping
    );

  rules[4].init(1);
  rules[4].setParameters(
    5, // type
    0.5, 1.0, // age range
    V3(-20, 2, -5), // min velocity
    V3(20, 18, 5), // max velocity
    0.01 // damping
    );
  rules[4].payloads[0]._set(3, 5);

  rules[5].init(0);
  rules[5].setParameters(
    6, // type
    3, 5, // age range
    V3(-5, 5, -5), // min velocity
    V3(5, 10, 5), // max velocity
    0.95 // damping
    );

  rules[6].init(1);
  rules[6].setParameters(
    7, // type
    4, 5, // age range
    V3(-5, 50, -5), // min velocity
    V3(5, 60, 5), // max velocity
    0.01 // damping
    );
  rules[6].payloads[0]._set(8, 10);

  rules[7].init(0);
  rules[7].setParameters(
    8, // type
    0.25, 0.5, // age range
    V3(-1, -1, -1), // min velocity
    V3(1, 1, 1), // max velocity
    0.01 // damping
    );

  rules[8].init(0);
  rules[8].setParameters(
    9, // type
    3, 5, // age range
    V3(-15, 10, -5), // min velocity
    V3(15, 15, 5), // max velocity
    0.95 // damping
    );
  // ... and so on for other firework types ...
End;

Constructor FireworksDemo.Create;
Var
  firework: Integer;
Begin
  Inherited Create();
  nextFirework := 0;
  // Make all shots unused
  For firework := 0 To high(fireworks) Do Begin
    fireworks[firework]._type := 0;
  End;

  // Create the firework types
  initFireworkRules();
End;

Function FireworksDemo.GetTitle: String;
Begin
  Result := 'Cyclone > Fireworks Demo';
End;

Procedure FireworksDemo.initGraphics;
Begin
  Inherited initGraphics();
  // But override the clear color
  glClearColor(0.0, 0.0, 0.1, 1.0);
End;

Procedure FireworksDemo.Key(akey: Char);
Begin
  Case akey Of
    '1': create(1, 1, Nil);
    '2': create(2, 1, Nil);
    '3': create(3, 1, Nil);
    '4': create(4, 1, Nil);
    '5': create(5, 1, Nil);
    '6': create(6, 1, Nil);
    '7': create(7, 1, Nil);
    '8': create(8, 1, Nil);
    '9': create(9, 1, Nil);
  End;
End;

Procedure FireworksDemo.Display;
Var
  size: float;
  pos: Vector3;
  firework: Integer;
Begin
  size := 0.1;

  // Clear the viewport and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(0.0, 4.0, 10.0, 0.0, 4.0, 0.0, 0.0, 1.0, 0.0);

  // Render each firework in turn
  glBegin(GL_QUADS);
  For firework := 0 To high(fireworks) Do Begin
    // Check if we need to process this firework.
    If (fireworks[firework]._type > 0) Then Begin
      Case (fireworks[firework]._type) Of
        1: glColor3f(1, 0, 0);
        2: glColor3f(1, 0.5, 0);
        3: glColor3f(1, 1, 0);
        4: glColor3f(0, 1, 0);
        5: glColor3f(0, 1, 1);
        6: glColor3f(0.4, 0.4, 1);
        7: glColor3f(1, 0, 1);
        8: glColor3f(1, 1, 1);
        9: glColor3f(1, 0.5, 0.5);
      End;

      pos := fireworks[firework].getPosition();
      glVertex3f(pos.x - size, pos.y - size, pos.z);
      glVertex3f(pos.x + size, pos.y - size, pos.z);
      glVertex3f(pos.x + size, pos.y + size, pos.z);
      glVertex3f(pos.x - size, pos.y + size, pos.z);

      // Render the firework's reflection
      glVertex3f(pos.x - size, -pos.y - size, pos.z);
      glVertex3f(pos.x + size, -pos.y - size, pos.z);
      glVertex3f(pos.x + size, -pos.y + size, pos.z);
      glVertex3f(pos.x - size, -pos.y + size, pos.z);
    End;
  End;
  glEnd();
End;

Procedure FireworksDemo.Update;
Var
  duration: float;
  rule, i, firework: Integer;
Begin
  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  For firework := 0 To high(fireworks) Do Begin
    // Check if we need to process this firework.
    If (fireworks[firework]._type > 0) Then Begin

      // Does it need removing?
      If (fireworks[firework].update(duration)) Then Begin

        // Find the appropriate rule
        rule := fireworks[firework]._type - 1;

        // Delete the current firework (this doesn't affect its
        // position and velocity for passing to the create function,
        // just whether or not it is processed for rendering or
        // physics.
        fireworks[firework]._type := 0;

        // Add the payload
        For i := 0 To rules[rule].payloadCount - 1 Do Begin
          create(rules[rule].payloads[i]._type, rules[rule].payloads[i].count, @fireworks[firework]);
        End;
      End;
    End;
  End;
  Inherited Update;
End;

End.

