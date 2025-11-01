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
Unit urandom;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucore, uprecision;

Type

  { TRandom }

  TRandom = Class
  private
    // Internal mechanics
    p1, p2: integer;
    buffer: Array[0..16] Of UInt32;
  public

    Constructor Create(); virtual;
    (**
     * left bitwise rotation
     *)

    //unsigned rotl(unsigned n, unsigned r);
    ///**
    // * right bitwise rotation
    // */
    //unsigned rotr(unsigned n, unsigned r);
    //
    //   /**
    //    * Creates a new random number stream with a seed based on
    //    * timing data.
    //    */
    //   Random();
    //
    //   /**
    //    * Creates a new random stream with the given seed.
    //    */
    //   Random(unsigned seed);
    //
       (**
        * Sets the seed value for the random stream.
        *)
    Procedure seed(s: UInt32);

    //   /**
    //    * Returns the next random bitstring from the stream. This is
    //    * the fastest method.
    //    */
    //   unsigned randomBits();
    //
       (**
        * Returns a random floating point number between 0 and 1.
        *)
    Function randomReal(): Float; overload;

    (**
     * Returns a random floating point number between 0 and scale.
     *)
    Function randomReal(scale: float): Float; overload;

    (**
     * Returns a random floating point number between min and max.
     *)
    Function randomReal(min, max: Float): Float; overload;

    (**
     * Returns a random integer less than the given value.
     *)
    Function randomInt(max: UInt32): UInt32;

    (**
     * Returns a random binomially distributed number between -scale
     * and +scale.
     *)
    Function randomBinomial(scale: float): float;

    (**
     * Returns a random vector where each component is binomially
     * distributed in the range (-scale to scale) [mean = 0.0f].
     *)
    Function randomVector(scale: float): Vector3; overload;

    (**
     * Returns a random vector where each component is binomially
     * distributed in the range (-scale to scale) [mean = 0.0f],
     * where scale is the corresponding component of the given
     * vector.
     *)
    Function randomVector(Const scale: Vector3): Vector3; overload;

    (**
     * Returns a random vector in the cube defined by the given
     * minimum and maximum vectors. The probability is uniformly
     * distributed in this region.
     *)
    Function randomVector(Const min, max: Vector3): Vector3; overload;

    (**
     * Returns a random vector where each component is binomially
     * distributed in the range (-scale to scale) [mean = 0.0f],
     * except the y coordinate which is zero.
     *)
    Function randomXZVector(scale: Float): Vector3;

    (**
     * Returns a random orientation (i.e. normalized) quaternion.
     *)
    Function randomQuaternion(): Quaternion;


  End;

Var
  Random: TRandom;

Implementation

{ TRandom }

Constructor TRandom.Create;
Begin
  seed(0);
End;

Procedure TRandom.seed(s: UInt32);
Var
  i: Integer;
Begin
  If (s = 0) Then Begin
    Randomize;
    s := system.random(High(integer));
  End;

  // Fill the buffer with some basic random numbers
  For i := 0 To 16 Do Begin
    // Simple linear congruential generator
    s := uint32(s * 2891336453 + 1);
    buffer[i] := s;
  End;

  // Initialize pointers into the buffer
  p1 := 0;
  p2 := 10;
End;

Function TRandom.randomReal: Float;
Begin
  result := system.Random;
End;

Function TRandom.randomReal(scale: float): Float;
Begin
  result := system.Random * scale;
End;

Function TRandom.randomReal(min, max: Float): Float;
Begin
  result := system.Random * (max - min) + min;
End;

Function TRandom.randomInt(max: UInt32): UInt32;
Begin
  result := system.Random(max);
End;

Function TRandom.randomBinomial(scale: float): float;
Begin
  result := (randomReal() - randomReal()) * scale;
End;

Function TRandom.randomVector(scale: float): Vector3;
Begin
  result.create(
    randomBinomial(scale),
    randomBinomial(scale),
    randomBinomial(scale)
    );
End;

Function TRandom.randomVector(Const scale: Vector3): Vector3;
Begin
  result.create(
    randomBinomial(scale.x),
    randomBinomial(scale.y),
    randomBinomial(scale.z)
    );
End;

Function TRandom.randomVector(Const min, max: Vector3): Vector3;
Begin
  result.create(
    randomReal(min.x, max.x),
    randomReal(min.y, max.y),
    randomReal(min.z, max.z)
    );
End;

Function TRandom.randomXZVector(scale: Float): Vector3;
Begin
  result.create(
    randomBinomial(scale),
    0,
    randomBinomial(scale)
    );
End;

Function TRandom.randomQuaternion: Quaternion;
Var
  q: Quaternion;
Begin
  q.create(
    randomReal(),
    randomReal(),
    randomReal(),
    randomReal()
    );
  q.normalise();
  result := q;
End;

Initialization
  Random := TRandom.Create;

Finalization
  random.free;
  random := Nil;

End.

