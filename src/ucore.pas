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
Unit ucore;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, uprecision;

Type

  { TArray }

  Generic TArray < T > = Class
  private
    Data: Array Of T;
    Function GetElement(index: integer): T;
    Procedure SetElement(index: integer; AValue: T);
  public
    Property Element[index: integer]: T read GetElement write SetElement; default;

    Constructor Create(); virtual;

    Function Item(index: integer): T;

    Procedure Push_back(aValue: T);
    Function Length: Integer;
  End;

  { Vector3 }

  Vector3 = Object
  private
    pad: float; // Not used at all, just to pad the size to 16 byte instead 12 when using float = single
  public
    x, y, z: float;

    Constructor create(); overload;
    Constructor create(aX, aY, aZ: float); overload;

    Function componentProduct(Const v: Vector3): Vector3;
    Procedure componentProductUpdate(Const v: Vector3);
    Function scalarProduct(Const v: Vector3): Float;
    Procedure addScaledVector(Const v: Vector3; s: Float);
    Function VectorProduct(Const v: Vector3): Vector3;
    Function Magnitude(): Float;
    Function squareMagnitude(): Float;
    Procedure Normalize;
    Procedure Invert;
    Procedure Clear;
  End;

  PVector3 = ^Vector3;

Operator * (v: Vector3; s: Float): Vector3;
Operator * (s: Float; v: Vector3): Vector3;
Operator * (a, b: Vector3): Float;
Operator Mod (a, b: Vector3): Vector3;

Operator + (a, b: Vector3): Vector3;
Operator - (a, b: Vector3): Vector3;

Var
  GRAVITY: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  HIGH_GRAVITY: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  UP: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  RIGHT: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  OUT_OF_SCREEN: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  X: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  Y: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.
  Z: Vector3; // ist eigentlich "Const", werden in Initialization gesetzt.

Procedure makeOrthonormalBasis(Var a, b: Vector3; Out c: Vector3);

Function V3(x, y, z: float): Vector3;

Implementation

Function V3(x, y, z: float): Vector3;
Begin
  result.create(x, y, z);
End;

Operator * (v: Vector3; s: Float): Vector3;
Begin
  result.x := v.x * s;
  result.y := v.y * s;
  result.z := v.z * s;
End;

Operator * (s: Float; v: Vector3): Vector3;
Begin
  result.x := v.x * s;
  result.y := v.y * s;
  result.z := v.z * s;
End;

Operator * (a, b: Vector3): Float;
Begin
  result := a.x * b.x + a.y * b.y + a.z * b.z;
End;

Operator Mod (a, b: Vector3): Vector3;
Begin
  result.create(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
    );
End;

Operator + (a, b: Vector3): Vector3;
Begin
  result.x := a.x + b.x;
  result.y := a.y + b.y;
  result.z := a.z + b.z;
End;

Operator - (a, b: Vector3): Vector3;
Begin
  result.x := a.x - b.x;
  result.y := a.y - b.y;
  result.z := a.z - b.z;
End;

Procedure makeOrthonormalBasis(Var a, b: Vector3; Out c: Vector3);
Begin
  a.Normalize;
  c := a Mod b;
  If c.squareMagnitude = 0 Then exit; // Error ?
  c.Normalize;
  b := c Mod a;
End;

{ TArray }

function TArray.GetElement(index: integer): T;
Begin
  result := item(index);
End;

procedure TArray.SetElement(index: integer; AValue: T);
Begin

End;

constructor TArray.Create;
Begin
  Data := Nil;
End;

function TArray.Item(index: integer): T;
Begin
  result := data[index];
End;

procedure TArray.Push_back(aValue: T);
Begin
  setlength(data, high(Data) + 2);
  data[high(Data)] := aValue;
End;

function TArray.Length: Integer;
begin
  result := system.length(data);
end;

{ Vector3 }

Constructor Vector3.create;
Begin
  x := 0;
  y := 0;
  z := 0;
End;

Constructor Vector3.create(aX, aY, aZ: float);
Begin
  x := aX;
  y := aY;
  z := aZ;
End;

Function Vector3.componentProduct(Const v: Vector3): Vector3;
Begin
  result.create(x * v.x, y * v.y, z * v.z);
End;

Procedure Vector3.componentProductUpdate(Const v: Vector3);
Begin
  x := x * v.x;
  y := y * v.y;
  z := z * v.z;
End;

Function Vector3.scalarProduct(Const v: Vector3): Float;
Begin
  result := v.x * x + v.y * y + v.z * z;
End;

Procedure Vector3.addScaledVector(Const v: Vector3; s: Float);
Begin
  x := x + v.x * s;
  y := y + v.y * s;
  z := z + v.z * s;
End;

Function Vector3.VectorProduct(Const v: Vector3): Vector3;
Begin
  result.create(
    y * v.z - z * v.y,
    z * v.x - x * v.z,
    x * v.y - y * v.x
    );
End;

Function Vector3.Magnitude: Float;
Begin
  result := sqrt(sqr(x) + sqr(y) + sqr(z));
End;

Function Vector3.squareMagnitude: Float;
Begin
  result := sqr(x) + sqr(y) + sqr(z);
End;

Procedure Vector3.Normalize;
Var
  l: Float;
Begin
  l := Magnitude();
  If (l > 0) Then Begin
    self := self * (1 / l);
  End;
End;

Procedure Vector3.Invert;
Begin
  x := -x;
  y := -y;
  z := -z;
End;

Procedure Vector3.Clear;
Begin
  x := 0;
  y := 0;
  z := 0;
End;

Initialization

  GRAVITY.create(0, -9.81, 0);
  HIGH_GRAVITY.create(0, -19.62, 0);
  UP.create(0, 1, 0);
  RIGHT.create(1, 0, 0);
  OUT_OF_SCREEN.create(0, 0, 1);
  x.create(0, 1, 0);
  y.create(1, 0, 0);
  z.create(0, 0, 1);


End.

