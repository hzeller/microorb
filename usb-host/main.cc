// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// Talk to the microorb. Commandline interface to class MicroOrb.
//
// This requires libusb (tested on Linux and MacOS)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "microorb.h"

using orb_driver::MicroOrb;

static int usage(const char *prog) {
  fprintf(stderr,
          "usage: %s [options] <RGB-Hex>[:<morph-ms>[:<hold-ms>]] [<RGB>...]\n"
          "Just supply a list of one or more hex colors like: FF0000\n"
          ".. or with transition times: FF0000:500:3000 00FF00:500:3000\n"
          "\n\thttp://go/microorb/manual\n\n"
          "Options:\n"
          " -s <orb-serial>   : Address a MicroOrb with a particular"
          " serial#.\n"
          "                     Only necessary if there are multiple Orbs"
          " attached.\n"
          " -l                : List all attached Orbs with serial# and"
          " capabilities.\n"
          " -g                : Get current color.\n"
          " -G                : Get current sequence if supported.\n"
          " -v                : Verbose.\n"
          " -x <1|on|0|off>   : Switch aux on/off\n",
          prog);
  return -1;
}

// Open first usable orb. If serial number is requested, with that serial
// number. Do up to three retries on failure.
static MicroOrb *OpenOrb(const char *request_serial) {
  const int kRetries = 8;
  for (int i = 0 ; i < kRetries; ++i) {
    MicroOrb::DeviceList devices;
    MicroOrb::UsbList(&devices);
    for (MicroOrb::DeviceList::const_iterator it = devices.begin();
         it != devices.end(); ++it) {
      MicroOrb *orb = MicroOrb::Open(*it);
      if (orb && (request_serial == NULL
                  || request_serial == orb->GetSerial())) {
        return orb;
      }
      delete orb;
    }
  }
  return NULL;
}

static bool ListAvailableOrbSerials() {
  MicroOrb::DeviceList devices;
  MicroOrb::UsbList(&devices);
  int count = 0;
  for (MicroOrb::DeviceList::const_iterator it = devices.begin();
       it != devices.end(); ++it) {
    struct orb_capabilities_t cap;
    MicroOrb *orb = MicroOrb::Open(*it);
    if (orb && orb->GetCapabilities(&cap)) {
      ++count;
      fprintf(stderr, "%s\tmax-sequence=%d\thas-capability=[%s]\tversion=%d\n",
              orb->GetSerial().c_str(), cap.max_sequence_len,
              MicroOrb::FormatCapabilitiesString(cap).c_str(), cap.version);
    }
    delete orb;
  }
  if (count == 0) {
    fprintf(stderr, "No orbs found.\n");
    return false;
  }
  return true;
}

// Parse color and duration spec and return number of parameters found. Does
// not touch morphtime or duration if not found.
// Color is defined as RRGGBB[:<optional-morph-ms>[:<optional-duration-ms]]
// The RRGGBB values are hex, the duration values can be up to 60000ms and
// will be quantized to 250ms.
static int ParseColorSpec(const char *str, struct orb_rgb_t *out,
                          int *morphtime, int *duration) {
  int r, g, b;
  int m, d;
  int params;
  if (strlen(str) < 6
      || (params = sscanf(str, "%02x%02x%02x:%d:%d",
                          &r, &g, &b, &m, &d)) < 3) {
    fprintf(stderr, "invalid color format '%s'.\n"
            "Use a hex value in the form RRGGBB such as '050FAB'.\n",
            str);
    return 0;
  }
  out->red = r;
  out->green = g;
  out->blue = b;
  if (params >= 4 && morphtime) *morphtime = m;
  if (params >= 5 && duration) *duration = d;
  return params;
}


// Parse a color sequence given on the command line and stuff it in an
// orb_sequence_t struct.
static bool ParseSequence(const char *const *args, int color_num,
                          struct orb_sequence_t *seq) {
  if (color_num > ORB_MAX_SEQUENCE) {
    color_num = ORB_MAX_SEQUENCE;
    fprintf(stderr, "Exceeded supported number of colors in sequence."
            " Limiting to %d\n", color_num);
  }
  seq->count = color_num;
  int i;
  int morph, duration;
  for (i = 0; i < color_num; ++i) {
    morph = 0;
    duration = 250;
    if (ParseColorSpec(args[i], &seq->period[i].color, &morph, &duration) < 3) {
      return false;
    }
    morph = (morph + 249) / 250;
    duration = (duration + 249) / 250;
    seq->period[i].morph_time = morph < 255 ? morph : 255;
    seq->period[i].hold_time = duration < 255 ? duration : 255;
  }
  return true;
}


// Parse an aux value (such as 'on' or 'off' or '1' or '0') from the commandline
// and stuff it in the aux_value result.
static bool ParseAuxValue(const char *arg, char *aux_value) {
  if (strcmp(arg, "on") == 0) {
    *aux_value = 1;
  } else if (strcmp(arg, "off") == 0) {
    *aux_value = 0;
  } else {
    char *err;
    *aux_value = strtol(optarg, &err, 10);
    if (*err) {
      fprintf(stderr, "invalid integer: '%s'\n", err);
      return false;
    }
    if ((*aux_value & 0x01) != *aux_value) {
      fprintf(stderr, "right now, only values 0 or 1\n");
      return false;
    }
  }
  return true;
}

static bool PrintOrbColor(MicroOrb *orb) {
  struct orb_rgb_t color;
  if (!orb->GetColor(&color))
    return false;
  fprintf(stdout, "%02x%02x%02x\n", color.red, color.green, color.blue);
  return true;
}

static void FormatSequence(const struct orb_sequence_t *seq) {
  int i;
  for (i = 0; i < seq->count; ++i) {
    const struct orb_color_period_t *period = &seq->period[i];
    fprintf(stderr, "%02x%02x%02x:%d:%d ",
            period->color.red, period->color.green, period->color.blue,
            period->morph_time * 250, period->hold_time * 250);
  }
  fprintf(stderr, "\n");
}

int main(int argc, char **argv) {
  enum Mode {
    SET_COLOR_SEQUENCE,
    GET_COLOR,
    GET_SEQUENCE,
    SET_AUX,
    LIST_DEVICES,
  } mode = SET_COLOR_SEQUENCE;

  int verbose = 0;
  char aux_value = 0;
  char *request_serial = NULL;

  int opt;
  while ((opt = getopt(argc, argv, "gGvhls:x:P:")) != -1) {
    switch (opt) {
      case 'l':
        mode = LIST_DEVICES;
        break;

      case 'g':
        mode = GET_COLOR;
        break;

      case 'G':
        mode = GET_SEQUENCE;
        break;

      case 's':
        request_serial = strdup(optarg);
        break;

      case 'h':
        usage(argv[0]);
        return 0;

      case 'v':
        verbose = 1;
        break;

      case 'x':
        mode = SET_AUX;
        if (!ParseAuxValue(optarg, &aux_value))
          return usage(argv[0]);
        break;

      default:
        return usage(argv[0]);
    }
  }

  const int kArgOffset = optind;
  const int arg_num = argc - kArgOffset;

  if (mode == SET_COLOR_SEQUENCE && arg_num == 0) {
      // No colors given, show help and list available orbs.
      usage(argv[0]);
      fprintf(stderr, "-- Connected orbs --\n");
      ListAvailableOrbSerials();
      return 1;
  }

  if (mode == LIST_DEVICES) {
    return ListAvailableOrbSerials() ? 0 : 1;
  }

  /* The other commands require to open the partiulcar orb */
  MicroOrb *orb = OpenOrb(request_serial);
  if (orb == NULL) {
    if (request_serial) {
      fprintf(stderr, "No orb with serial '%s' found.\n", request_serial);
    } else {
      fprintf(stderr, "No orb found.\n");
    }
    return 1;
  }

  if (mode == GET_COLOR) {
    return PrintOrbColor(orb) ? 0 : 1;
  } else if (mode == GET_SEQUENCE) {
    struct orb_sequence_t seq;
    if (!orb->GetSequence(&seq)) {
      return 1;
    }
    FormatSequence(&seq);
  } else if (mode == SET_AUX) {
    return orb->SetAux(aux_value) ? 0 : 1;
  } else {
    if (arg_num == 0) {
      return usage(argv[0]);
    }

    struct orb_sequence_t seq;
    if (!ParseSequence(argv + kArgOffset, arg_num, &seq))
      usage(argv[0]);
    if (verbose) FormatSequence(&seq);
    orb->SetSequence(seq);
  }
  delete orb;
  return 0;
}
