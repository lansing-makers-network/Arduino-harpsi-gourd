#include "PitchToNote.h"

/*
Piano keys Left to Right
 F3, G3b,  G3, A3b,  A3, B3b,  B3,  C4, D4b,  D4, E4b,  E4,  F4, G4b,  G4, A4b,  A4, B4b,  B4,  C5, D5b,  D5, E5b,  E5
  1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24
*/

mprs chips[] = {
    (mprs) {
      MPR121A, // pointer to above reserved memory structure
      0x5A,    // ADDR tied to GND, individual address of MPR121 on I2C bus
      {30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30}, //tthresh
      {10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10}, //rthresh
      {00,  00,  00,  00,  00,  00,  00,  00,  00,  00,  00,  00}, //timeout
      { 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, //noteState
      {F3, G3b,  G3, A3b,  A3, B3b,  B3,  C4, D4b,  D4, E4b,  E4}, //key    ** Customize to fit your Build **
      { 1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12}  //ledPos ** Customize to fit your Build **
    },

    (mprs) {
      MPR121B, // pointer to above reserved memory structure
      0x5B,    // ADDR tied to 5V, individual address of MPR121 on I2C bus
      {30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30,  30}, //tthresh
      {10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10,  10}, //rthresh
      {00,  00,  00,  00,  00,  00,  00,  00,  00,  00,  00,  00}, //timeout
      { 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0}, //noteState
      {F4, G4b,  G4, A4b,  A4, B4b,  B4,  C5, D5b,  D5, E5b,  E5}, //key    ** Customize to fit your Build **
      {13,  14,  15,  16,  17,  18,  19,  20,  21,  22,  23,  24}  //ledPos ** Customize to fit your Build **
    }
  };