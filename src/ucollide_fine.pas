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
Unit ucollide_fine;

{$MODE ObjFPC}{$H+}

Interface

Uses
  Classes, SysUtils, ucontacts;

Type
  (**
   * A helper structure that contains information for the detector to use
   * in building its contact data.
   *)
  CollisionData = Object
    (**
     * Holds the base of the collision data: the first contact
     * in the array. This is used so that the contact pointer (below)
     * can be incremented each time a contact is detected, while
     * this pointer points to the first contact found.
     *)
    contactArray: pContact;
    //
    //           /** Holds the contact array to write into. */
    //           Contact *contacts;
    //
    //           /** Holds the maximum number of contacts the array can take. */
    //           int contactsLeft;
    //
    //           /** Holds the number of contacts found so far. */
    //           unsigned contactCount;
    //
    //           /** Holds the friction value to write into any collisions. */
    //           real friction;
    //
    //           /** Holds the restitution value to write into any collisions. */
    //           real restitution;
    //
    //           /**
    //            * Holds the collision tolerance, even uncolliding objects this
    //            * close should have collisions generated.
    //            */
    //           real tolerance;
    //
    //           /**
    //            * Checks if there are more contacts available in the contact
    //            * data.
    //            */
    //           bool hasMoreContacts()
    //           {
    //               return contactsLeft > 0;
    //           }
    //
    //           /**
    //            * Resets the data so that it has no used contacts recorded.
    //            */
    //           void reset(unsigned maxContacts)
    //           {
    //               contactsLeft = maxContacts;
    //               contactCount = 0;
    //               contacts = contactArray;
    //           }
    //
    //           /**
    //            * Notifies the data that the given number of contacts have
    //            * been added.
    //            */
    //           void addContacts(unsigned count)
    //           {
    //               // Reduce the number of contacts remaining, add number used
    //               contactsLeft -= count;
    //               contactCount += count;
    //
    //               // Move the array forward
    //               contacts += count;
    //           }
  End;

Implementation

End.

