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

Const
  (*
   * Definition of the sleep epsilon extern.
   *)
  sleepEpsilon = 0.3;

Type

  { TArray }

  Generic TArray < T > = Class
  private
    Data: Array Of T;
    Function getCount: integer;
    Function GetElement(index: integer): T;
    Procedure SetElement(index: integer; AValue: T);
  public
    Property Element[index: integer]: T read GetElement write SetElement; default;
    Property Count: integer read getCount;

    Constructor Create(); virtual;

    Function Item(index: integer): T;

    Procedure Push_back(aValue: T);
    Function Length: Integer;
    Function _Begin: T;
    Function _End: T;
  End;

  TOpenGLMatrix = Array[0..15] Of Single;

  { Vector3 }

  Vector3 = Object
  private
    pad: float; // Not used at all, just to pad the size to 16 byte instead 12 when using float = single
    Function getIndex(aIndex: integer): float;
    Procedure setIndex(aIndex: integer; AValue: float);
  public
    x, y, z: float;

    Property Index[aIndex: integer]: float read getIndex write setIndex; default;

    Constructor create(); overload;
    Constructor create(aX, aY, aZ: float); overload;

    Function componentProduct(Const v: Vector3): Vector3;
    Procedure componentProductUpdate(Const v: Vector3);
    Function scalarProduct(Const v: Vector3): Float;
    Procedure addScaledVector(Const v: Vector3; s: Float);
    Function VectorProduct(Const v: Vector3): Vector3;
    Function Magnitude(): Float;
    Function squareMagnitude(): Float;
    (** Limits the size of the vector to the given maximum. *)
    Procedure trim(size: FLoat);
    (** Turns a non-zero vector into a vector of unit length. *)
    Procedure Normalize;
    (** Returns the normalised version of a vector. *)
    Function _Unit: Vector3;
    Procedure Invert;
    Procedure Clear;
  End;

  PVector3 = ^Vector3;

  (**
    * Holds a three degree of freedom orientation.
    *
    * Quaternions have
    * several mathematical properties that make them useful for
    * representing orientations, but require four items of data to
    * hold the three degrees of freedom. These four items of data can
    * be viewed as the coefficients of a complex number with three
    * imaginary parts. The mathematics of the quaternion is then
    * defined and is roughly correspondent to the math of 3D
    * rotations. A quaternion is only a valid rotation if it is
    * normalised: i.e. it has a length of 1.
    *
    * @note Angular velocity and acceleration can be correctly
    * represented as vectors. Quaternions are only needed for
    * orientation.
    *)

  TQuaternionInternal = Record
    Case boolean Of
      true: (
        (**
         * Holds the real component of the quaternion.
         *)
        r: Float;
        (**
         * Holds the first complex component of the
         * quaternion.
         *)
        i: Float;
        (**
         * Holds the second complex component of the
         * quaternion.
         *)
        j: Float;
        (**
         * Holds the third complex component of the
         * quaternion.
         *)
        k: Float;
        );
      (**
       * Holds the quaternion data in array form.
       *)
      false: (data: Array[0..3] Of Float);
  End;

  { Quaternion }

  Quaternion = Object
  public
    _d: TQuaternionInternal;
    (**
     * The default constructor creates a quaternion representing
     * a zero rotation.
     *)
    Constructor Create;

    (**
     * The explicit constructor creates a quaternion with the given
     * components.
     *
     * @param r The real component of the rigid body's orientation
     * quaternion.
     *
     * @param i The first complex component of the rigid body's
     * orientation quaternion.
     *
     * @param j The second complex component of the rigid body's
     * orientation quaternion.
     *
     * @param k The third complex component of the rigid body's
     * orientation quaternion.
     *
     * @note The given orientation does not need to be normalised,
     * and can be zero. This function will not alter the given
     * values, or normalise the quaternion. To normalise the
     * quaternion (and make a zero quaternion a legal rotation),
     * use the normalise function.
     *
     * @see normalise
     *)
    Constructor Create(r, i, j, k: FLoat);

    (**
     * Normalises the quaternion to unit length, making it a valid
     * orientation quaternion.
     *)
    Procedure normalise();

    (**
     * Adds the given vector to this, scaled by the given amount.
     * This is used to update the orientation quaternion by a rotation
     * and time.
     *
     * @param vector The vector to add.
     *
     * @param scale The amount of the vector to add.
     *)
    Procedure addScaledVector(Const vector: Vector3; scale: float);

    //        void rotateByVector(const Vector3& vector)
    //        {
    //            Quaternion q(0, vector.x, vector.y, vector.z);
    //            (*this) *= q;
    //        }
  End;

  (**
   * Holds a transform matrix, consisting of a rotation matrix and
   * a position. The matrix has 12 elements, it is assumed that the
   * remaining four are (0,0,0,1); producing a homogenous matrix.
   *)

  { Matrix4 }

  Matrix4 = Object
  public
    (**
     * Holds the transform matrix data in array form.
     *)
    data: Array[0..11] Of float;

    // ... Other Matrix4 code as before ...

    (**
     * Creates an identity matrix.
     *)
    Constructor create();
    //
    //         /**
    //          * Sets the matrix to be a diagonal matrix with the given coefficients.
    //          */
    //         void setDiagonal(real a, real b, real c)
    //         {
    //             data[0] = a;
    //             data[5] = b;
    //             data[10] = c;
    //         }
    //         /**
    //          * Transform the given vector by this matrix.
    //          *
    //          * @param vector The vector to transform.
    //          */
    //         Vector3 operator*(const Vector3 &vector) const
    //         {
    //             return Vector3(
    //                 vector.x * data[0] +
    //                 vector.y * data[1] +
    //                 vector.z * data[2] + data[3],
    //
    //                 vector.x * data[4] +
    //                 vector.y * data[5] +
    //                 vector.z * data[6] + data[7],
    //
    //                 vector.x * data[8] +
    //                 vector.y * data[9] +
    //                 vector.z * data[10] + data[11]
    //             );
    //         }
    //
             (**
              * Transform the given vector by this matrix.
              *
              * @param vector The vector to transform.
              *)
    Function transform(Const vector: Vector3): Vector3;
    //
    //         /**
    //          * Returns the determinant of the matrix.
    //          */
    //         real getDeterminant() const;
    //
    //         /**
    //          * Sets the matrix to be the inverse of the given matrix.
    //          *
    //          * @param m The matrix to invert and use to set this.
    //          */
    //         void setInverse(const Matrix4 &m);
    //
    //         /** Returns a new matrix containing the inverse of this matrix. */
    //         Matrix4 inverse() const
    //         {
    //             Matrix4 result;
    //             result.setInverse(*this);
    //             return result;
    //         }
    //
    //         /**
    //          * Inverts the matrix.
    //          */
    //         void invert()
    //         {
    //             setInverse(*this);
    //         }

             (**
              * Transform the given direction vector by this matrix.
              *
              * @note When a direction is converted between frames of
              * reference, there is no translation required.
              *
              * @param vector The vector to transform.
              *)
    Function transformDirection(Const vector: Vector3): vector3;

    (**
     * Transform the given direction vector by the
     * transformational inverse of this matrix.
     *
     * @note This function relies on the fact that the inverse of
     * a pure rotation matrix is its transpose. It separates the
     * translational and rotation components, transposes the
     * rotation, and multiplies out. If the matrix is not a
     * scale and shear free transform matrix, then this function
     * will not give correct results.
     *
     * @note When a direction is converted between frames of
     * reference, there is no translation required.
     *
     * @param vector The vector to transform.
     *)
    Function transformInverseDirection(Const vector: Vector3): Vector3;

    (**
     * Transform the given vector by the transformational inverse
     * of this matrix.
     *
     * @note This function relies on the fact that the inverse of
     * a pure rotation matrix is its transpose. It separates the
     * translational and rotation components, transposes the
     * rotation, and multiplies out. If the matrix is not a
     * scale and shear free transform matrix, then this function
     * will not give correct results.
     *
     * @param vector The vector to transform.
     *)
    Function transformInverse(Const vector: Vector3): Vector3;

    (**
     * Gets a vector representing one axis (i.e. one column) in the matrix.
     *
     * @param i The row to return. Row 3 corresponds to the position
     * of the transform matrix.
     *
     * @return The vector.
     *)
    Function getAxisVector(i: integer): Vector3;

    //         /**
    //          * Sets this matrix to be the rotation matrix corresponding to
    //          * the given quaternion.
    //          */
    //         void setOrientationAndPos(const Quaternion &q, const Vector3 &pos)
    //         {
    //             data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
    //             data[1] = 2*q.i*q.j + 2*q.k*q.r;
    //             data[2] = 2*q.i*q.k - 2*q.j*q.r;
    //             data[3] = pos.x;
    //
    //             data[4] = 2*q.i*q.j - 2*q.k*q.r;
    //             data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
    //             data[6] = 2*q.j*q.k + 2*q.i*q.r;
    //             data[7] = pos.y;
    //
    //             data[8] = 2*q.i*q.k + 2*q.j*q.r;
    //             data[9] = 2*q.j*q.k - 2*q.i*q.r;
    //             data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
    //             data[11] = pos.z;
    //         }

             (**
              * Fills the given array with this transform matrix, so it is
              * usable as an open-gl transform matrix. OpenGL uses a column
              * major format, so that the values are transposed as they are
              * written.
              *)
    Procedure fillGLArray(Out _array: TOpenGLMatrix);
  End;

  PMatrix4 = ^Matrix4;

  (**
    * Holds an inertia tensor, consisting of a 3x3 row-major matrix.
    * This matrix is not padding to produce an aligned structure, since
    * it is most commonly used with a mass (single real) and two
    * damping coefficients to make the 12-element characteristics array
    * of a rigid body.
    *)

  { Matrix3 }

  Matrix3 = Object
  public
    (**
     * Holds the tensor matrix data in array form.
     *)
    data: Array[0..8] Of float;

    // ... Other Matrix3 code as before ...

    (**
     * Creates a new matrix.
     *)
    Constructor Create();

    (**
     * Creates a new matrix with the given three vectors making
     * up its columns.
     *)
//        Matrix3(const Vector3 &compOne, const Vector3 &compTwo,
//            const Vector3 &compThree)
//        {
//            setComponents(compOne, compTwo, compThree);
//        }
//
//        /**
//         * Creates a new matrix with explicit coefficients.
//         */
//        Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
//            real c6, real c7, real c8)
//        {
//            data[0] = c0; data[1] = c1; data[2] = c2;
//            data[3] = c3; data[4] = c4; data[5] = c5;
//            data[6] = c6; data[7] = c7; data[8] = c8;
//        }

        (**
         * Sets the matrix to be a diagonal matrix with the given
         * values along the leading diagonal.
         *)
    Procedure setDiagonal(a, b, c: float);

    (**
     * Sets the value of the matrix from inertia tensor values.
     *)
    Procedure setInertiaTensorCoeffs(ix, iy, iz: Float;
      ixy: Float = 0; ixz: Float = 0; iyz: Float = 0);

    (**
     * Sets the value of the matrix as an inertia tensor of
     * a rectangular block aligned with the body's coordinate
     * system with the given axis half-sizes and mass.
     *)
    Procedure setBlockInertiaTensor(Const halfSizes: Vector3; mass: Float);

    (**
     * Sets the matrix to be a skew symmetric matrix based on
     * the given vector. The skew symmetric matrix is the equivalent
     * of the vector product. So if a,b are vectors. a x b = A_s b
     * where A_s is the skew symmetric form of a.
     *)
    Procedure setSkewSymmetric(Const vector: Vector3);

    (**
     * Sets the matrix values from the given three vector components.
     * These are arranged as the three columns of the vector.
     *)
    Procedure setComponents(Const compOne: Vector3; Const compTwo: Vector3;
      Const compThree: Vector3);

    (**
     * Transform the given vector by this matrix.
     *
     * @param vector The vector to transform.
     *)
    Function transform(Const vector: Vector3): Vector3;

    (**
     * Transform the given vector by the transpose of this matrix.
     *
     * @param vector The vector to transform.
     *)
    Function transformTranspose(Const vector: Vector3): Vector3;

    //        /**
    //         * Gets a vector representing one row in the matrix.
    //         *
    //         * @param i The row to return.
    //         */
    //        Vector3 getRowVector(int i) const
    //        {
    //            return Vector3(data[i*3], data[i*3+1], data[i*3+2]);
    //        }
    //
    //        /**
    //         * Gets a vector representing one axis (i.e. one column) in the matrix.
    //         *
    //         * @param i The row to return.
    //         *
    //         * @return The vector.
    //         */
    //        Vector3 getAxisVector(int i) const
    //        {
    //            return Vector3(data[i], data[i+3], data[i+6]);
    //        }
    //
            (**
             * Sets the matrix to be the inverse of the given matrix.
             *
             * @param m The matrix to invert and use to set this.
             *)
    Procedure setInverse(Const m: Matrix3);


    (** Returns a new matrix containing the inverse of this matrix. *)
    Function inverse(): Matrix3;

    //        /**
    //         * Inverts the matrix.
    //         */
    //        void invert()
    //        {
    //            setInverse(*this);
    //        }

            (**
             * Sets the matrix to be the transpose of the given matrix.
             *
             * @param m The matrix to transpose and use to set this.
             *)
    Procedure setTranspose(Const m: Matrix3);

    (** Returns a new matrix containing the transpose of this matrix. *)
    Function transpose(): Matrix3;

    //        /**
    //         * Multiplies this matrix in place by the given other matrix.
    //         */
    //        void operator*=(const Matrix3 &o)
    //        {
    //            real t1;
    //            real t2;
    //            real t3;
    //
    //            t1 = data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6];
    //            t2 = data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7];
    //            t3 = data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8];
    //            data[0] = t1;
    //            data[1] = t2;
    //            data[2] = t3;
    //
    //            t1 = data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6];
    //            t2 = data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7];
    //            t3 = data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8];
    //            data[3] = t1;
    //            data[4] = t2;
    //            data[5] = t3;
    //
    //            t1 = data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6];
    //            t2 = data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7];
    //            t3 = data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8];
    //            data[6] = t1;
    //            data[7] = t2;
    //            data[8] = t3;
    //        }
    //
    //        /**
    //         * Sets this matrix to be the rotation matrix corresponding to
    //         * the given quaternion.
    //         */
    //        void setOrientation(const Quaternion &q)
    //        {
    //            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
    //            data[1] = 2*q.i*q.j + 2*q.k*q.r;
    //            data[2] = 2*q.i*q.k - 2*q.j*q.r;
    //            data[3] = 2*q.i*q.j - 2*q.k*q.r;
    //            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
    //            data[5] = 2*q.j*q.k + 2*q.i*q.r;
    //            data[6] = 2*q.i*q.k + 2*q.j*q.r;
    //            data[7] = 2*q.j*q.k - 2*q.i*q.r;
    //            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
    //        }
  End;

Operator * (v: Vector3; s: Float): Vector3;
Operator * (s: Float; v: Vector3): Vector3;
Operator * (a, b: Vector3): Float;
Operator * (m, o: Matrix4): Matrix4;
Operator * (m: Matrix3; vector: Vector3): Vector3;
Operator * (m: Matrix4; vector: Vector3): Vector3;
Operator * (Const s, o: Matrix3): Matrix3;
Operator + (Const s, o: Matrix3): Matrix3;

Operator * (Const m: Matrix3; Const scalar: float): Matrix3;


Operator * (Const a, b: Quaternion): Quaternion;

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

Function Q(r, i, j, k: FLoat): Quaternion;
Function V3(x, y, z: float): Vector3;
Function M3(a, b, c, d, e, f, g, h, i: float): Matrix3;

Function ifthen(value: Boolean; truecase, falsecase: Float): Float; // ka warum math.pas includiert und alles knallt ..

Procedure Nop;

(**
 * Interpolates a couple of matrices.
 *)
Function linearInterpolate(Const a, b: Matrix3; prop: float): Matrix3;

Implementation

Procedure Nop;
Begin

End;

Function linearInterpolate(Const a, b: Matrix3; prop: float): Matrix3;
Var
  i: integer;
Begin
  For i := 0 To 8 Do Begin
    result.data[i] := a.data[i] * (1 - prop) + b.data[i] * prop;
  End;
End;

Function ifthen(value: Boolean; truecase, falsecase: Float): Float;
Begin
  If value Then
    result := truecase
  Else
    result := falsecase;
End;

Function Q(r, i, j, k: FLoat): Quaternion;
Begin
  result.Create(r, i, j, k);
End;

Function V3(x, y, z: float): Vector3;
Begin
  result.create(x, y, z);
End;

Function M3(a, b, c, d, e, f, g, h, i: float): Matrix3;
Begin
  result.data[0] := a;
  result.data[1] := b;
  result.data[2] := c;
  result.data[3] := d;
  result.data[4] := e;
  result.data[5] := f;
  result.data[6] := g;
  result.data[7] := h;
  result.data[8] := i;
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

(**
 * Returns a matrix which is this matrix multiplied by the given
 * other matrix.
 *)

Operator * (m, o: Matrix4): Matrix4;
Begin
  result.data[0] := (o.data[0] * m.data[0]) + (o.data[4] * m.data[1]) + (o.data[8] * m.data[2]);
  result.data[4] := (o.data[0] * m.data[4]) + (o.data[4] * m.data[5]) + (o.data[8] * m.data[6]);
  result.data[8] := (o.data[0] * m.data[8]) + (o.data[4] * m.data[9]) + (o.data[8] * m.data[10]);

  result.data[1] := (o.data[1] * m.data[0]) + (o.data[5] * m.data[1]) + (o.data[9] * m.data[2]);
  result.data[5] := (o.data[1] * m.data[4]) + (o.data[5] * m.data[5]) + (o.data[9] * m.data[6]);
  result.data[9] := (o.data[1] * m.data[8]) + (o.data[5] * m.data[9]) + (o.data[9] * m.data[10]);

  result.data[2] := (o.data[2] * m.data[0]) + (o.data[6] * m.data[1]) + (o.data[10] * m.data[2]);
  result.data[6] := (o.data[2] * m.data[4]) + (o.data[6] * m.data[5]) + (o.data[10] * m.data[6]);
  result.data[10] := (o.data[2] * m.data[8]) + (o.data[6] * m.data[9]) + (o.data[10] * m.data[10]);

  result.data[3] := (o.data[3] * m.data[0]) + (o.data[7] * m.data[1]) + (o.data[11] * m.data[2]) + m.data[3];
  result.data[7] := (o.data[3] * m.data[4]) + (o.data[7] * m.data[5]) + (o.data[11] * m.data[6]) + m.data[7];
  result.data[11] := (o.data[3] * m.data[8]) + (o.data[7] * m.data[9]) + (o.data[11] * m.data[10]) + m.data[11];
End;

(**
 * Transform the given vector by this matrix.
 *
 * @param vector The vector to transform.
 *)

Operator * (m: Matrix3; vector: Vector3): Vector3;
Begin
  result.create(
    vector.x * m.data[0] + vector.y * m.data[1] + vector.z * m.data[2],
    vector.x * m.data[3] + vector.y * m.data[4] + vector.z * m.data[5],
    vector.x * m.data[6] + vector.y * m.data[7] + vector.z * m.data[8]
    );
End;

(**
 * Transform the given vector by this matrix.
 *
 * @param vector The vector to transform.
 *)

Operator * (m: Matrix4; vector: Vector3): Vector3;
Begin
  result.create(
    vector.x * m.data[0] +
    vector.y * m.data[1] +
    vector.z * m.data[2] + m.data[3],

    vector.x * m.data[4] +
    vector.y * m.data[5] +
    vector.z * m.data[6] + m.data[7],

    vector.x * m.data[8] +
    vector.y * m.data[9] +
    vector.z * m.data[10] + m.data[11]
    );
End;

(**
 * Returns a matrix which is this matrix multiplied by the given
 * other matrix.
 *)

Operator * (Const s, o: Matrix3): Matrix3;
Begin
  result.data[0] := s.data[0] * o.data[0] + s.data[1] * o.data[3] + s.data[2] * o.data[6];
  result.data[1] := s.data[0] * o.data[1] + s.data[1] * o.data[4] + s.data[2] * o.data[7];
  result.data[2] := s.data[0] * o.data[2] + s.data[1] * o.data[5] + s.data[2] * o.data[8];

  result.data[3] := s.data[3] * o.data[0] + s.data[4] * o.data[3] + s.data[5] * o.data[6];
  result.data[4] := s.data[3] * o.data[1] + s.data[4] * o.data[4] + s.data[5] * o.data[7];
  result.data[5] := s.data[3] * o.data[2] + s.data[4] * o.data[5] + s.data[5] * o.data[8];

  result.data[6] := s.data[6] * o.data[0] + s.data[7] * o.data[3] + s.data[8] * o.data[6];
  result.data[7] := s.data[6] * o.data[1] + s.data[7] * o.data[4] + s.data[8] * o.data[7];
  result.data[8] := s.data[6] * o.data[2] + s.data[7] * o.data[5] + s.data[8] * o.data[8];
End;

(**
 * Does a component-wise addition of this matrix and the given
 * matrix.
 *)

Operator + (Const s, o: Matrix3): Matrix3;
Begin
  result.data[0] := s.data[0] + o.data[0];
  result.data[1] := s.data[1] + o.data[1];
  result.data[2] := s.data[2] + o.data[2];
  result.data[3] := s.data[3] + o.data[3];
  result.data[4] := s.data[4] + o.data[4];
  result.data[5] := s.data[5] + o.data[5];
  result.data[6] := s.data[6] + o.data[6];
  result.data[7] := s.data[7] + o.data[7];
  result.data[8] := s.data[8] + o.data[8];
End;

(**
 * Multiplies this matrix in place by the given scalar.
 *)

Operator * (Const m: Matrix3; Const scalar: float): Matrix3;
Begin
  result.data[0] := m.data[0] * scalar;
  result.data[1] := m.data[1] * scalar;
  result.data[2] := m.data[2] * scalar;
  result.data[3] := m.data[3] * scalar;
  result.data[4] := m.data[4] * scalar;
  result.data[5] := m.data[5] * scalar;
  result.data[6] := m.data[6] * scalar;
  result.data[7] := m.data[7] * scalar;
  result.data[8] := m.data[8] * scalar;
End;

(**
 * Multiplies the quaternion by the given quaternion.
 *
 * @param multiplier The quaternion by which to multiply.
 *)

Operator * (Const a, b: Quaternion): Quaternion;
Begin
  result._d.r := a._d.r * b._d.r - a._d.i * b._d.i -
    a._d.j * b._d.j - a._d.k * b._d.k;
  result._d.i := a._d.r * b._d.i + a._d.i * b._d.r +
    a._d.j * b._d.k - a._d.k * b._d.j;
  result._d.j := a._d.r * b._d.j + a._d.j * b._d.r +
    a._d.k * b._d.i - a._d.i * b._d.k;
  result._d.k := a._d.r * b._d.k + a._d.k * b._d.r +
    a._d.i * b._d.j - a._d.j * b._d.i;
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

Function TArray.GetElement(index: integer): T;
Begin
  result := item(index);
End;

Function TArray.getCount: integer;
Begin
  result := system.length(Data);
End;

Procedure TArray.SetElement(index: integer; AValue: T);
Begin
  data[index] := AValue;
End;

Constructor TArray.Create;
Begin
  Data := Nil;
End;

Function TArray.Item(index: integer): T;
Begin
  result := data[index];
End;

Procedure TArray.Push_back(aValue: T);
Begin
  setlength(data, high(Data) + 2);
  data[high(Data)] := aValue;
End;

Function TArray.Length: Integer;
Begin
  result := system.length(data);
End;

Function TArray._Begin: T;
Begin
  result := data[0];
End;

Function TArray._End: T;
Begin
  result := data[high(data)];
End;

{ Vector3 }

Function Vector3.getIndex(aIndex: integer): float;
Begin
  Case aIndex Of
    0: result := x;
    1: result := y;
  Else
    result := z;
  End;
End;

Procedure Vector3.setIndex(aIndex: integer; AValue: float);
Begin
  Case aIndex Of
    0: x := AValue;
    1: y := AValue;
  Else
    z := AValue;
  End;
End;

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

Procedure Vector3.trim(size: FLoat);
Begin
  If (squareMagnitude() > size * size) Then Begin
    Normalize();
    x := x * size;
    y := y * size;
    z := z * size;
  End;
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

Function Vector3._Unit: Vector3;
Begin
  result := self;
  result.Normalize();
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

{ Quaternion }

Constructor Quaternion.Create;
Begin
  _d.r := 1;
  _d.i := 0;
  _d.j := 0;
  _d.k := 0;
End;

Constructor Quaternion.Create(r, i, j, k: FLoat);
Begin
  _d.r := r;
  _d.i := i;
  _d.j := j;
  _d.k := k;
End;

Procedure Quaternion.normalise;
Var
  d: float;
Begin
  d := _d.r * _d.r + _d.i * _d.i + _d.j * _d.j + _d.k * _d.k;

  // Check for zero length quaternion, and use the no-rotation
  // quaternion in that case.
  If (d < real_epsilon) Then Begin
    _d.r := 1;
    exit;
  End;

  d := (1.0) / real_sqrt(d);
  _d.r := _d.r * d;
  _d.i := _d.i * d;
  _d.j := _d.j * d;
  _d.k := _d.k * d;
End;

Procedure Quaternion.addScaledVector(Const vector: Vector3; scale: float);
Var
  q: Quaternion;
Begin
  q.Create(0,
    vector.x * scale,
    vector.y * scale,
    vector.z * scale);
  q := q * self;
  _d.r := _d.r + q._d.r * 0.5;
  _d.i := _d.i + q._d.i * 0.5;
  _d.j := _d.j + q._d.j * 0.5;
  _d.k := _d.k + q._d.k * 0.5;
End;

{ Matrix4 }

Constructor Matrix4.create;
Begin
  data[0] := 1;
  data[1] := 0;
  data[2] := 0;
  data[3] := 0;
  data[4] := 0;
  data[5] := 1;
  data[6] := 0;
  data[7] := 0;
  data[8] := 0;
  data[9] := 0;
  data[10] := 1;
  data[11] := 0;
End;

Function Matrix4.transform(Const vector: Vector3): Vector3;
Begin
  result := self * vector;
End;

Function Matrix4.transformDirection(Const vector: Vector3): vector3;
Begin
  result.create(
    vector.x * data[0] +
    vector.y * data[1] +
    vector.z * data[2],

    vector.x * data[4] +
    vector.y * data[5] +
    vector.z * data[6],

    vector.x * data[8] +
    vector.y * data[9] +
    vector.z * data[10]
    );
End;

Function Matrix4.transformInverseDirection(Const vector: Vector3): Vector3;
Begin
  result.create(
    vector.x * data[0] +
    vector.y * data[4] +
    vector.z * data[8],

    vector.x * data[1] +
    vector.y * data[5] +
    vector.z * data[9],

    vector.x * data[2] +
    vector.y * data[6] +
    vector.z * data[10]
    );
End;

Function Matrix4.transformInverse(Const vector: Vector3): Vector3;
Var
  tmp: Vector3;
Begin
  tmp := vector;
  tmp.x := tmp.x - data[3];
  tmp.y := tmp.y - data[7];
  tmp.z := tmp.z - data[11];
  result.create(
    tmp.x * data[0] +
    tmp.y * data[4] +
    tmp.z * data[8],

    tmp.x * data[1] +
    tmp.y * data[5] +
    tmp.z * data[9],

    tmp.x * data[2] +
    tmp.y * data[6] +
    tmp.z * data[10]
    );
End;

Function Matrix4.getAxisVector(i: integer): Vector3;
Begin
  result.create(data[i], data[i + 4], data[i + 8]);
End;

Procedure Matrix4.fillGLArray(Out _array: TOpenGLMatrix);
Begin
  _array[0] := data[0];
  _array[1] := data[4];
  _array[2] := data[8];
  _array[3] := 0;

  _array[4] := data[1];
  _array[5] := data[5];
  _array[6] := data[9];
  _array[7] := 0;

  _array[8] := data[2];
  _array[9] := data[6];
  _array[10] := data[10];
  _array[11] := 0;

  _array[12] := data[3];
  _array[13] := data[7];
  _array[14] := data[11];
  _array[15] := 1;
End;

{ Matrix3 }

Constructor Matrix3.Create;
Begin
  data[0] := 1;
  data[1] := 0;
  data[2] := 0;

  data[3] := 0;
  data[4] := 1;
  data[5] := 0;

  data[6] := 0;
  data[7] := 0;
  data[8] := 1;
End;

Procedure Matrix3.setDiagonal(a, b, c: float);
Begin
  setInertiaTensorCoeffs(a, b, c);
End;

Procedure Matrix3.setInertiaTensorCoeffs(ix, iy, iz: Float; ixy: Float;
  ixz: Float; iyz: Float);
Begin
  data[0] := ix;
  data[1] := -ixy;
  data[3] := -ixy;
  data[2] := -ixz;
  data[6] := -ixz;
  data[4] := iy;
  data[5] := -iyz;
  data[7] := -iyz;
  data[8] := iz;
End;

Procedure Matrix3.setBlockInertiaTensor(Const halfSizes: Vector3; mass: Float);
Var
  squares: Vector3;
Begin
  squares := halfSizes.componentProduct(halfSizes);
  setInertiaTensorCoeffs(0.3 * mass * (squares.y + squares.z),
    0.3 * mass * (squares.x + squares.z),
    0.3 * mass * (squares.x + squares.y));
End;

Procedure Matrix3.setSkewSymmetric(Const vector: Vector3);
Begin
  data[0] := 0;
  data[4] := 0;
  data[8] := 0;
  data[1] := -vector.z;
  data[2] := vector.y;
  data[3] := vector.z;
  data[5] := -vector.x;
  data[6] := -vector.y;
  data[7] := vector.x;
End;

Procedure Matrix3.setComponents(Const compOne: Vector3; Const compTwo: Vector3;
  Const compThree: Vector3);
Begin
  data[0] := compOne.x;
  data[1] := compTwo.x;
  data[2] := compThree.x;
  data[3] := compOne.y;
  data[4] := compTwo.y;
  data[5] := compThree.y;
  data[6] := compOne.z;
  data[7] := compTwo.z;
  data[8] := compThree.z;
End;

Function Matrix3.transform(Const vector: Vector3): Vector3;
Begin
  result := self * vector;
End;

Function Matrix3.transformTranspose(Const vector: Vector3): Vector3;
Begin
  result.create(
    vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
    vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
    vector.x * data[2] + vector.y * data[5] + vector.z * data[8]
    );
End;

Procedure Matrix3.setInverse(Const m: Matrix3);
Var
  t4, t6, t8, t10, t12, t14, t16, t17: float;
Begin
  t4 := m.data[0] * m.data[4];
  t6 := m.data[0] * m.data[5];
  t8 := m.data[1] * m.data[3];
  t10 := m.data[2] * m.data[3];
  t12 := m.data[1] * m.data[6];
  t14 := m.data[2] * m.data[6];

  // Calculate the determinant
  t16 := (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
    t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

  // Make sure the determinant is non-zero.
  If (t16 = 0.0) Then exit;
  t17 := 1 / t16;

  data[0] := (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
  data[1] := -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
  data[2] := (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
  data[3] := -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
  data[4] := (m.data[0] * m.data[8] - t14) * t17;
  data[5] := -(t6 - t10) * t17;
  data[6] := (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
  data[7] := -(m.data[0] * m.data[7] - t12) * t17;
  data[8] := (t4 - t8) * t17;
End;

Function Matrix3.inverse: Matrix3;
Begin
  result.setInverse(self);
End;

Procedure Matrix3.setTranspose(Const m: Matrix3);
Begin
  data[0] := m.data[0];
  data[1] := m.data[3];
  data[2] := m.data[6];
  data[3] := m.data[1];
  data[4] := m.data[4];
  data[5] := m.data[7];
  data[6] := m.data[2];
  data[7] := m.data[5];
  data[8] := m.data[8];
End;

Function Matrix3.transpose: Matrix3;
Begin
  result.setTranspose(self);
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

