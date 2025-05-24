Unit TestCase1;

{$MODE objfpc}{$H+}

Interface

Uses
  Classes, SysUtils, fpcunit, testutils, testregistry, core, precision;

Type

  { TCore }

  TCore = Class(TTestCase)
  protected
    Procedure SetUp; override;
    Procedure TearDown; override;
  published
    Procedure CreateZero;
    Procedure CreateWithComponents;

    Procedure ComponentProduct;
    Procedure ComponentProductUpdate;
    Procedure scalarProduct;
    Procedure addScaledVector;
    Procedure VectorProduct;
    Procedure Magnitude;
    Procedure squareMagnitude;
    Procedure Normalize;
    Procedure Invert;
    Procedure MakeOrthonormalBasis;
    Procedure Mul;
    Procedure Add;
    Procedure Sub;

  End;

Implementation

Const
  Epsilon = 1 / 256;

Procedure TCore.CreateZero;
Var
  v: Vector3;
Begin
  v.create();
  AssertTrue(v.x = 0);
  AssertTrue(v.y = 0);
  AssertTrue(v.z = 0);
End;

Procedure TCore.CreateWithComponents;
Var
  v: Vector3;
Begin
  v.create(1, 2, 3);
  AssertTrue(v.x = 1);
  AssertTrue(v.y = 2);
  AssertTrue(v.z = 3);
End;

Procedure TCore.ComponentProduct;
Var
  a, b, c: Vector3;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  c := a.componentProduct(b);
  AssertTrue(c.x = 1 * 4);
  AssertTrue(c.y = 2 * 5);
  AssertTrue(c.z = 3 * 6);
End;

Procedure TCore.ComponentProductUpdate;
Var
  a, b: Vector3;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  a.componentProductUpdate(b);
  AssertTrue(a.x = 1 * 4);
  AssertTrue(a.y = 2 * 5);
  AssertTrue(a.z = 3 * 6);
End;

Procedure TCore.scalarProduct;
Var
  a, b: Vector3;
  c: real;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  c := a.scalarProduct(b);
  AssertTrue(c = 1 * 4 + 2 * 5 + 3 * 6);
End;

Procedure TCore.addScaledVector;
Var
  a, b: Vector3;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  a.addScaledVector(b, 2);
  AssertTrue(a.x = 1 + 4 * 2);
  AssertTrue(a.y = 2 + 5 * 2);
  AssertTrue(a.z = 3 + 6 * 2);
End;

Procedure TCore.VectorProduct;
Var
  a, b, c: Vector3;
Begin
  a.create(1, 0, 0);
  b.create(0, 1, 0);
  c := a.VectorProduct(b);
  AssertTrue(c.x = 0);
  AssertTrue(c.y = 0);
  AssertTrue(c.z = 1);
  c := b.VectorProduct(a);
  AssertTrue(c.x = 0);
  AssertTrue(c.y = 0);
  AssertTrue(c.z = -1);
End;

Procedure TCore.Magnitude;
Var
  a: Vector3;
Begin
  a.create(1, 0, 0);
  AssertTrue(a.Magnitude() = 1);
  a.create(0, 1, 0);
  AssertTrue(a.Magnitude() = 1);
  a.create(0, 0, 1);
  AssertTrue(a.Magnitude() = 1);
End;

Procedure TCore.squareMagnitude;
Var
  a: Vector3;
Begin
  a.create(1, 0, 0);
  AssertTrue(a.squareMagnitude() = 1);
  a.create(1, 2, 3);
  AssertTrue(a.squareMagnitude() = 1 * 1 + 2 * 2 + 3 * 3);
End;

Procedure TCore.Normalize;
Var
  a: Vector3;
Begin
  a.create(10, 0, 0);
  a.Normalize;
  AssertTrue(a.x = 1);
  AssertTrue(a.y = 0);
  AssertTrue(a.z = 0);
  a.create(1, 1, 0);
  a.Normalize;
  AssertTrue((a.x - 1 / sqrt(2)) < Epsilon);
  AssertTrue((a.y - 1 / sqrt(2)) < Epsilon);
  AssertTrue((a.z - 0) < Epsilon);

End;

Procedure TCore.Invert;
Var
  a: Vector3;
Begin
  a.create(1, 2, 3);
  a.Invert;
  AssertTrue(a.x = -1);
  AssertTrue(a.y = -2);
  AssertTrue(a.z = -3);
End;

Procedure TCore.MakeOrthonormalBasis;
Var
  a, b, c: Vector3;
Begin
  a.create(1, 0, 0);
  b.create(1, 1, 0);
  core.makeOrthonormalBasis(a, b, c);
  AssertTrue(a.x = 1);
  AssertTrue(a.y = 0);
  AssertTrue(a.z = 0);
  AssertTrue(b.x = 0);
  AssertTrue(b.y = 1);
  AssertTrue(b.z = 0);
  AssertTrue(c.x = 0);
  AssertTrue(c.y = 0);
  AssertTrue(c.z = 1);

  a.create(10, 0, 0);
  b.create(10, 5, 0);
  core.makeOrthonormalBasis(a, b, c);
  AssertTrue(a.x = 1);
  AssertTrue(a.y = 0);
  AssertTrue(a.z = 0);
  AssertTrue(b.x = 0);
  AssertTrue(b.y = 1);
  AssertTrue(b.z = 0);
  AssertTrue(c.x = 0);
  AssertTrue(c.y = 0);
  AssertTrue(c.z = 1);
End;

Procedure TCore.Mul;
Var
  a, b: Vector3;
Begin
  a.create(1, 2, 3);
  b := a * 2;
  AssertTrue(b.x = 1 * 2);
  AssertTrue(b.y = 2 * 2);
  AssertTrue(b.z = 3 * 2);
  b := 3 * a;
  AssertTrue(b.x = 1 * 3);
  AssertTrue(b.y = 2 * 3);
  AssertTrue(b.z = 3 * 3);
End;

Procedure TCore.Add;
Var
  a, b, c: Vector3;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  c := a + b;
  AssertTrue(c.x = 1 + 4);
  AssertTrue(c.y = 2 + 5);
  AssertTrue(c.z = 3 + 6);
  c := a + a;
  AssertTrue(c.x = 1 * 2);
  AssertTrue(c.y = 2 * 2);
  AssertTrue(c.z = 3 * 2);
End;

Procedure TCore.Sub;
Var
  a, b, c: Vector3;
Begin
  a.create(1, 2, 3);
  b.create(4, 5, 6);
  c := a - b;
  AssertTrue(c.x = 1 - 4);
  AssertTrue(c.y = 2 - 5);
  AssertTrue(c.z = 3 - 6);
  c := a - a;
  AssertTrue(c.x = 0);
  AssertTrue(c.y = 0);
  AssertTrue(c.z = 0);
End;

Procedure TCore.SetUp;
Begin

End;

Procedure TCore.TearDown;
Begin

End;

Initialization

  RegisterTest(TCore);
End.

