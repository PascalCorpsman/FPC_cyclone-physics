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
Unit uApp;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils

  // cyclone.h
  , uprecision
  , ucore
  , urandom
  , uparticle
  //  #include "body.h"
  , upcontacts
  , upworld
  , ucollide_fine
  , ucontacts
  //  #include "fgen.h"
  //  #include "joints.h"
  ;

Const
  // State for mouse Event
  GLUT_UP = 0;
  GLUT_DOWN = 1;

Type

  { Application }

  Application = Class
  private
    width, Height: integer;
  public
    Constructor Create(); virtual;

    Function GetWindowDimension: TPoint; virtual;

    Function GetTitle: String; virtual;
    Procedure initGraphics(); virtual;
    Procedure setView(); virtual;

    Procedure resize(aWidth, aHeight: integer);

    Procedure Display; virtual;
    Procedure Key(akey: Char); virtual;
    Procedure Update; virtual;
    Procedure RenderText(x, y: Single; text: String);

    Procedure mouse(button, state, x, y: integer); virtual;
  End;


  { MassAggregateApplication }

  MassAggregateApplication = Class(Application)
  private
  protected
    world: ParticleWorld;
    particleArray: Array Of Particle;
    groundContactGenerator: GroundContacts;
  public
    Constructor Create(particleCount: integer); virtual; reintroduce;
    Destructor Destroy(); override;

    Procedure Update; override;
    Procedure initGraphics(); override;
    Procedure Display; override;
  End;

Const
  (** Holds the maximum number of contacts. *)
  maxContacts = 256;

Type

  { RigidBodyApplication }

  RigidBodyApplication = Class(Application)
  private
  protected

    (** Holds the array of contacts. *)
    contacts: Array[0..maxContacts - 1] Of Contact;

    (** Holds the collision data structure for collision detection. *)
    cData: CollisionData;

    (** Holds the contact resolver. *)
    resolver: ContactResolver;

    (** Holds the camera angle. *)
    theta: float;

    (** Holds the camera elevation. *)
    phi: float;

    (** Holds the position of the mouse at the last frame of a drag. *)
    last_x, last_y: integer;

    (** True if the contacts should be rendered. *)
    renderDebugInfo: Boolean;

    (** True if the simulation is paused. *)
    pauseSimulation: Boolean;

    (** Pauses the simulation after the next frame automatically *)
    autoPauseSimulation: Boolean;

    (** Processes the contact generation code. *)
    Procedure generateContacts(); virtual;

    (** Processes the objects in the simulation forward in time. *)
    Procedure updateObjects(duration: real); virtual;

    (**
     * Finishes drawing the frame, adding debugging information
     * as needed.
     *)
    Procedure drawDebug;

    (** Resets the simulation. *)
    Procedure reset; virtual;

  public
    (**
     * Creates a new application object.
     *)
    Constructor Create(); override;

    (** Display the application. *)
    Procedure display(); override;

    (** Update the objects. *)
    Procedure update(); override;

    //      /** Handle a mouse click. */
    //      virtual void mouse(int button, int state, int x, int y);
    //
    //      /** Handle a mouse drag */
    //      virtual void mouseDrag(int x, int y);
    //
    //      /** Handles a key press. */
    //      virtual void key(unsigned char key);

  End;

  // Created using ChatGTP, as glut is deprecated ...
Procedure glutSolidSphere(radius: Double; slices, stacks: integer);
Procedure glutSolidCube(size: Single);

Implementation

Uses dglOpenGL, uOpenGL_ASCII_Font, uvectormath, utiming;

Procedure glutSolidSphere(radius: Double; slices, stacks: integer);
Var
  i, j: Integer;
  lat0, lat1, z0, z1, zr0, zr1, lng, x, y: GLfloat;
Begin
  For i := 0 To stacks - 1 Do Begin
    lat0 := Pi * (-0.5 + i / stacks);
    z0 := Sin(lat0);
    zr0 := Cos(lat0);

    lat1 := Pi * (-0.5 + (i + 1) / stacks);
    z1 := Sin(lat1);
    zr1 := Cos(lat1);

    glBegin(GL_TRIANGLE_STRIP);
    For j := 0 To slices Do Begin
      lng := 2 * Pi * j / slices;
      x := Cos(lng);
      y := Sin(lng);

      glNormal3f(x * zr0, y * zr0, z0);
      glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);

      glNormal3f(x * zr1, y * zr1, z1);
      glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
    End;
    glEnd();
  End;
End;

Procedure glutSolidCube(size: Single);
Var
  s: GLfloat;
Begin
  s := size / 2.0;

  glBegin(GL_QUADS);

  // Front face  (z = +s)
  glNormal3f(0.0, 0.0, 1.0);
  glVertex3f(-s, -s, s);
  glVertex3f(s, -s, s);
  glVertex3f(s, s, s);
  glVertex3f(-s, s, s);

  // Back face (z = -s)
  glNormal3f(0.0, 0.0, -1.0);
  glVertex3f(-s, -s, -s);
  glVertex3f(-s, s, -s);
  glVertex3f(s, s, -s);
  glVertex3f(s, -s, -s);

  // Left face (x = -s)
  glNormal3f(-1.0, 0.0, 0.0);
  glVertex3f(-s, -s, -s);
  glVertex3f(-s, -s, s);
  glVertex3f(-s, s, s);
  glVertex3f(-s, s, -s);

  // Right face (x = +s)
  glNormal3f(1.0, 0.0, 0.0);
  glVertex3f(s, -s, -s);
  glVertex3f(s, s, -s);
  glVertex3f(s, s, s);
  glVertex3f(s, -s, s);

  // Top face (y = +s)
  glNormal3f(0.0, 1.0, 0.0);
  glVertex3f(-s, s, -s);
  glVertex3f(-s, s, s);
  glVertex3f(s, s, s);
  glVertex3f(s, s, -s);

  // Bottom face (y = -s)
  glNormal3f(0.0, -1.0, 0.0);
  glVertex3f(-s, -s, -s);
  glVertex3f(s, -s, -s);
  glVertex3f(s, -s, s);
  glVertex3f(-s, -s, s);

  glEnd();
End;

{ Application }

Constructor Application.Create;
Begin
  width := 1;
  Height := 1;
End;

Function Application.GetWindowDimension: TPoint;
Begin
  result := point(640, 480);
End;

Function Application.GetTitle: String;
Begin
  result := 'Cyclone Demo';
End;

Procedure Application.initGraphics;
Begin
  glClearColor(0.9, 0.95, 1.0, 1.0);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);

  setView();
End;

Procedure Application.setView;
Begin
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, width / height, 1.0, 500.0);
  glMatrixMode(GL_MODELVIEW);
End;

Procedure Application.resize(aWidth, aHeight: integer);
Begin
  // Avoid the divide by zero.
  If (height <= 0) Then height := 1;

  // Set the internal variables and update the view
  width := awidth;
  height := aheight;
  glViewport(0, 0, width, height);
  setView();
End;

Procedure Application.Display;
Begin
  glClear(GL_COLOR_BUFFER_BIT);

  glBegin(GL_LINES);
  glVertex2i(1, 1);
  glVertex2i(639, 319);
  glEnd();
End;

Procedure Application.Key(akey: Char);
Begin

End;

Procedure Application.Update;
Begin

End;

Procedure Application.RenderText(x, y: Single; text: String);
Var
  v: TVector4;
Begin
  glGetFloatv(GL_CURRENT_COLOR, @v.x);
  glDisable(GL_DEPTH_TEST);
  glBindTexture(GL_TEXTURE_2D, 0);
  Go2d(width, Height);
  OpenGL_ASCII_Font.ColorV3 := uvectormath.v3(v.x, v.y, v.z);
  OpenGL_ASCII_Font.Textout(round(x), round(height - y), text);
  Exit2d();
  glEnable(GL_DEPTH_TEST);
End;

Procedure Application.mouse(button, state, x, y: integer);
Begin

End;

{ MassAggregateApplication }

Constructor MassAggregateApplication.Create(particleCount: integer);
Var
  i: Integer;
Begin
  Inherited Create();
  world := ParticleWorld.Create(particleCount * 10);
  setlength(particleArray, particleCount);
  For i := 0 To particleCount - 1 Do Begin
    world.getParticles().Push_back(@particleArray[i]);
  End;
  groundContactGenerator := GroundContacts.Create;
  groundContactGenerator.init(world.getParticles());
  world.getContactGenerators().push_back(groundContactGenerator);
End;

Destructor MassAggregateApplication.Destroy();
Begin
  world.free;
  world := Nil;
  setlength(particleArray, 0);
  particleArray := Nil;
  groundContactGenerator.free;
  groundContactGenerator := Nil;
End;

Procedure MassAggregateApplication.initGraphics();
Begin
  Inherited initGraphics();
End;

Procedure MassAggregateApplication.Display;
Var
  Particles: TParticles;
  particle: PParticle;
  iterator: Integer;
  pos: Vector3;
Begin
  // Clear the view port and set the camera direction
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(0.0, 3.5, 8.0, 0.0, 3.5, 0.0, 0.0, 1.0, 0.0);

  glColor3f(0, 0, 0);

  particles := world.getParticles();
  For iterator := 0 To Particles.Count - 1 Do Begin
    particle := Particles[iterator];
    pos := particle^.getPosition();
    glPushMatrix();
    glTranslatef(pos.x, pos.y, pos.z);
    glutSolidSphere(0.1, 20, 10);
    glPopMatrix();
  End;
End;

Procedure MassAggregateApplication.Update;
Var
  duration: float;
Begin
  // Clear accumulators
  world.startFrame();

  // Find the duration of the last frame in seconds
  duration := TimingData.lastFrameDuration * 0.001;
  If (duration <= 0.0) Then exit;

  // Run the simulation
  world.runPhysics(duration);

  Inherited update();
End;

{ RigidBodyApplication }

Procedure RigidBodyApplication.generateContacts;
Begin

End;

Procedure RigidBodyApplication.updateObjects(duration: real);
Begin

End;

Procedure RigidBodyApplication.drawDebug;
Begin

End;

Procedure RigidBodyApplication.reset;
Begin

End;

Constructor RigidBodyApplication.Create;
Begin
  Inherited Create();

  theta := 0.0;
  phi := 15.0;
  //    resolver(maxContacts*8),
  //
  //    renderDebugInfo(false),
  //    pauseSimulation(true),
  //    autoPauseSimulation(false)
  //    cData.contactArray = contacts;

End;

Procedure RigidBodyApplication.display;
Begin
  glClear(GL_COLOR_BUFFER_BIT Or GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  gluLookAt(18.0, 0, 0, 0, 0, 0, 0, 1.0, 0);
  glRotatef(-phi, 0, 0, 1);
  glRotatef(theta, 0, 1, 0);
  glTranslatef(0, -5.0, 0);
End;

Procedure RigidBodyApplication.update();
Begin
  hier weiter
  // Find the duration of the last frame in seconds
//  float duration = (float)TimingData::get().lastFrameDuration * 0.001f;
//  if (duration <= 0.0f) return;
//  else if (duration > 0.05f) duration = 0.05f;
//
//  // Exit immediately if we aren't running the simulation
//  if (pauseSimulation)
//  {
//      Application::update();
//      return;
//  }
//  else if (autoPauseSimulation)
//  {
//      pauseSimulation = true;
//      autoPauseSimulation = false;
//  }
//
//  // Update the objects
//  updateObjects(duration);
//
//  // Perform the contact generation
//  generateContacts();
//
//  // Resolve detected contacts
//  resolver.resolveContacts(
//      cData.contactArray,
//      cData.contactCount,
//      duration
//      );
//
//  Application::update();
End;

End.


