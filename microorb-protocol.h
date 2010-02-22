// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// GPL license.
//
// Shared data structures for the protocol between firmware and hostware.

#ifndef MICROORB_PROTOCOL_H
#define MICROORB_PROTOCOL_H

#define ORB_MAX_SEQUENCE 16

// Magic value to switch off current limit.
#define ORB_SWITCH_CURRENT_LIMIT_OFF_MAGIC 0x5a

enum OrbCapabilityFlags {
  HAS_GET_COLOR    = 0x01,
  HAS_GET_SEQUENCE = 0x02,
  HAS_AUX          = 0x04,
  HAS_GAMMA_CORRECT= 0x08,
  HAS_CURRENT_LIMIT= 0x10,
  /* more to come */
};

struct orb_capabilities_t {
  unsigned char flags;
  unsigned char max_sequence_len;
  unsigned char version;
  unsigned char reserved2;
};

enum OrbRequest {
  ORB_SETCOLOR,
  ORB_GETCAPABILITIES,
  ORB_SETAUX,
  ORB_GETCOLOR,
  ORB_GETSEQUENCE,
  ORB_POKE_EEPROM,
};

struct orb_rgb_t {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

// One color with fade-in time and hold time. Element of a sequence.
struct orb_color_period_t {
  struct orb_rgb_t rgb;
  unsigned char morph_time;  // time to morph to the this color in 250ms steps
  unsigned char hold_time;   // time to hold this color in 250ms steps
};

// Sequence of colors sent to the Orb for it to iterate through.
struct orb_sequence_t {
  unsigned char count;
  struct orb_color_period_t period[ ORB_MAX_SEQUENCE ];
};

#endif /* MICROORB_PROTOCOL_H */
