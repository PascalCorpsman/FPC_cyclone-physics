program fpcunitproject1;

{$mode objfpc}{$H+}

uses
  Interfaces, Forms, GuiTestRunner, TestCase1, core;

{$R *.res}

begin
  Application.Initialize;
  Application.CreateForm(TGuiTestRunner, TestRunner);
  Application.Run;
end.

