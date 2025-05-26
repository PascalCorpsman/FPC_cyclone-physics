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
  Classes, SysUtils;

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

Procedure glutSolidSphere(radius: Double; slices, stacks: integer);

Implementation

Uses dglOpenGL, uOpenGL_ASCII_Font, uvectormath;

// Created using ChatGTP, as glut is deprecated ...

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
  OpenGL_ASCII_Font.ColorV3 := v3(v.x, v.y, v.z);
  OpenGL_ASCII_Font.Textout(round(x), round(height - y), text);
  Exit2d();
  glEnable(GL_DEPTH_TEST);
End;

Procedure Application.mouse(button, state, x, y: integer);
Begin

End;

End.

