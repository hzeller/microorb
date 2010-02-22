// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// GPL license.
//
// Shared data structures for the protocol between firmware and hostware.
//
// Protocol
// The protocol between the orb and the host on the other end of the USB
// are commands to be exchanged over usb control messages, with the command
// request value as one of enum OrbRequest.
//
// --- ORB_SETSEQUENCE (host -> orb). Always implemented. ---
// The protocol for setting a sequence of colors is the number of sequence
// elements followed by 5 byte structs containing the color and times.
// ----------
// struct sequence_t {
//  uchar sequence_elements;
//  struct {
//      uchar red;
//      uchar green;
//      uchar blue;
//      uchar morph_time;  // time to morph to the this color in 250ms steps
//      uchar hold_time;   // time to hold this color in 250ms steps
//  } sequence[sequence_elements];
// }
// (essentially a serialized orb_sequence_t, but truncated to the number of
//  sequence_elements)
//  -----------
//  The whole sequence is repeated by the orb as soon as the data has been
//  transmitted.
//  If the sequence only contains one element, then hold_time and morph_time
//  are effectivly irrelevant.
//
// --- ORB_GETCAPABILITIES (orb -> host). Always implemented. ---
// Returns a struct orb_capabilities_t that contains flags that describe the
// capabilities of this orb and the number of elements it allows in a sequence.
//
// --- ORB_SETAUX (host -> orb). Only implemented if HAS_AUX. --
// If capability HAS_AUX, then the single written byte corresponds to the
// bits on the aux port.
//
// --- ORB_GETCOLOR (orb -> host). Only implemented if HAS_GET_COLOR. ---
// Get the currently displayed color. If this orb is morphing between two
// colors, then this is the current intermediate color. Returns three bytes
// with red, green, blue.  (struct orb_rgb_t)
//
// --- ORB_GETSEQUENCE (orb -> host). Only implemented if HAS_GET_SEQUENCE. ---
// Returns the full sequence currently being handled in the same format as
// used in ORB_SET_SEQUENCE (dynamically sized struct orb_sequence_t)

#ifndef MICROORB_PROTOCOL_H
#define MICROORB_PROTOCOL_H

#define ORB_MAX_SEQUENCE 16

// Magic value to switch off current limit. Implementation private.
#define ORB_SWITCH_CURRENT_LIMIT_OFF_MAGIC 0x5a

enum OrbCapabilityFlags {    // Does the orb ...
  HAS_GET_COLOR    = 0x01,   // ... have the GETCOLOR operation implemented ?
  HAS_GET_SEQUENCE = 0x02,   // ... have the GET_SEQUENCE operation implemented?
  HAS_AUX          = 0x04,   // ... have an AUX port and SETAUX implemented ?
  HAS_GAMMA_CORRECT= 0x08,   // ... have built-in gamma correction ?
  HAS_CURRENT_LIMIT= 0x10,   // ... behave well on 500mA USB port ?
  /* more to come */
};

struct orb_capabilities_t {
  unsigned char flags;            // OR'ed set of OrbCapabilityFlags
  unsigned char max_sequence_len; // maximum number of colors in sequence
  unsigned char version;          // Firmware version (0 and 1 for now)
  unsigned char reserved2;        // yeah, fixed size format, you know ...
};

// USB control message request (described in header documentation on top)
enum OrbRequest {
  ORB_SETSEQUENCE,
  ORB_GETCAPABILITIES,
  ORB_SETAUX,
  ORB_GETCOLOR,
  ORB_GETSEQUENCE,
  ORB_POKE_EEPROM,
};

// A struct used to hold a RGB color.
struct orb_rgb_t {
  unsigned char red;
  unsigned char green;
  unsigned char blue;
};

// A sequence element consists of the color, the time to morph to that color
// and the time to hold the color.
struct orb_color_period_t {
  struct orb_rgb_t color;
  unsigned char morph_time;  // time to morph to the this color in 250ms steps
  unsigned char hold_time;   // time to hold this color in 250ms steps
};

// Sequence of colors sent to the Orb for it to iterate through. This struct
// is sent over the wire dynamically sized: the 'count' determines how many
// orb_color_period_t structs follow.
struct orb_sequence_t {
  unsigned char count;
  struct orb_color_period_t period[ ORB_MAX_SEQUENCE ];
};

#endif /* MICROORB_PROTOCOL_H */
