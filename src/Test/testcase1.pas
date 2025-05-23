Unit TestCase1;

{$MODE objfpc}{$H+}

Interface

Uses
  Classes, SysUtils, fpcunit, testutils, testregistry, core;

Type

  { TCore }

  TCore = Class(TTestCase)
  protected
    Procedure SetUp; override;
    Procedure TearDown; override;
  published
    Procedure CreateZero;
    Procedure CreateWithComponents;
  End;

Implementation

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

Procedure TCore.SetUp;
Begin

End;

Procedure TCore.TearDown;
Begin

End;

Initialization

  RegisterTest(TCore);
End.

