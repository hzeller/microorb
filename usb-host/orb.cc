// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// Talk to the microorb.
// GPL license.
// This requires libusb (tested on Linux and MacOS)

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

#include <usb.h>

/* a typical place for the udev configuration file */
#define UDEV_CONFIG_FILE "/etc/udev/rules.d/40-permissions.rules"

/* The unix group we want to give access to the orb */
#define ORB_GROUP "eng"

/* we use as vendor 'Prototype product Vendor ID' */
#define ORB_VENDOR  0x6666
#define ORB_PRODUCT 0xF00D

static const int kMaxSequenceLen = 16;  // Max number of colors sent to Orb.
static const int kUsbTimeoutMs = 1500;
static const int kUsbRetries = 3;       // Retries in case of usb bus error.

enum CapabilityFlags {
  HAS_GET_COLOR    = 0x01,
  HAS_GET_SEQUENCE = 0x02,
  HAS_AUX          = 0x04,
  HAS_GAMMA_CORRECT= 0x08,
  HAS_CURRENT_LIMIT= 0x10,
  /* more to come */
};

struct capabilities_t {
  unsigned char flags;
  unsigned char max_sequence_len;
  unsigned char version;
  unsigned char reserved2;
};

struct rgb_t {
  unsigned char r;
  unsigned char g;
  unsigned char b;
};

// One color with fade-in time and hold time. Element of a sequence.
struct color_period_t {
  struct rgb_t rgb;
  unsigned char morph_time;  // time to morph to the this color in 250ms steps
  unsigned char hold_time;   // time to hold this color in 250ms steps
};

// Sequence of colors sent to the Orb for it to iterate through.
struct sequence_t {
  unsigned char count;
  struct color_period_t period[ kMaxSequenceLen ];
};

enum Requests {
  ORB_SETCOLOR,
  ORB_GETCAPABILITIES,
  ORB_SETAUX,
  ORB_GETCOLOR,
  ORB_GETSEQUENCE,
  ORB_POKE_EEPROM,
};

static void print_usb_error(int err) {
  fprintf(stderr, "libusb: %s\n", strerror(err));
  if (err == EPERM) {
    fprintf(stderr, "For permission problems, add the following line "
            "to %s\n#\n"
            "SUBSYSTEM==\"usb_device\", "
            "SYSFS{idVendor}==\"%04x\", SYSFS{idProduct}==\"%04x\", "
            "GROUP=\"%s\", MODE=\"660\"\n#\n"
            ".. then unplug and re-connect the orb\n",
            UDEV_CONFIG_FILE, ORB_VENDOR, ORB_PRODUCT, ORB_GROUP);
  }
}

static bool send_orb(usb_dev_handle *handle, enum Requests command,
                     const void* input, size_t data_len) {
  int result = -1;
  char *const data = const_cast<char*>(reinterpret_cast<const char*>(input));
  for (int i = 0; result < 0 && i < kUsbRetries; ++i) {
    result = usb_control_msg(handle,
                             USB_ENDPOINT_OUT
                             | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                             command, 0, 0,
                             data, data_len,
                             kUsbTimeoutMs);
  }
  if (result < 0) {
    print_usb_error(errno);
  }
  return result >= 0;
}

// Receive data from orb and store it in given buffer. Buffer is reset.
static bool receive_orb(usb_dev_handle *handle, enum Requests command,
                        void* buffer, size_t buffer_size) {
  int result = -1;
  for (int i = 0; result < 0 && i < kUsbRetries; ++i) {
    memset(buffer, 0, buffer_size);
    result = usb_control_msg(handle,
                             USB_ENDPOINT_IN
                             | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                             command, 0, 0,
                             reinterpret_cast<char*>(buffer), buffer_size,
                             kUsbTimeoutMs);
  }
  if (result < 0) {
    print_usb_error(errno);
  }
  return result >= 0;
}

static bool get_capabilities(usb_dev_handle *handle,
                             struct capabilities_t *capabilities) {
  return receive_orb(handle, ORB_GETCAPABILITIES, capabilities,
                     sizeof(*capabilities));
}

static bool print_orb_color(usb_dev_handle *handle) {
  struct rgb_t color;
  if (!receive_orb(handle, ORB_GETCOLOR, &color, sizeof(color)))
    return false;
  fprintf(stdout, "%02x%02x%02x\n", color.r, color.g, color.b);
  return true;
}

static bool set_aux(usb_dev_handle *handle, char value) {
  return send_orb(handle, ORB_SETAUX, &value, sizeof(value));
}

int hex_nibble(char d) {
  if (d >= '0' && d <= '9') return d - '0';
  if (d >= 'A' && d <= 'F') return d - 'A' + 10;
  if (d >= 'a' && d <= 'f') return d - 'a' + 10;
  return -1;
}

// Returns the hex-char or -1 if failed to read.
int hex_char(const char *s) {
  int upper = hex_nibble(*s++);
  if (upper < 0) return -1;
  int lower = hex_nibble(*s);
  if (lower < 0) return -1;
  return upper << 4 | lower;
}

// data format: AALLDD...  AA: address LL:len DD:data all hex.
static bool poke_eeprom(usb_dev_handle *handle, const char* data) {
  const int raw_len = strlen(data);
  if (raw_len < 6) {
    fprintf(stderr, "Invalid hex input; expected AALLDD....\n");
    return false;
  }
  const int address = hex_char(data);
  if (address < 0) {
    fprintf(stderr, "Invalid address\n");
    return false;
  }
  data += 2;
  const int data_len = hex_char(data);
  data += 2;
  if (data_len < 0) {
    fprintf(stderr, "Invalid data-len\n");
    return false;
  }
  if (data_len != (raw_len - 4) / 2) {
    fprintf(stderr, "Expected exactly %d bytes\n", data_len);
    return false;
  }
  if (data_len > 7) {
    fprintf(stderr, "Can at most write 7 bytes\n");
    return false;
  }
  char poke_data[256];
  poke_data[0] = address;
  int i;
  fprintf(stderr, "@ addr=%d write %d bytes : ", address, data_len);
  for (i = 0; i < data_len; ++i, data += 2) {
    int value = hex_char(data);
    if (value < 0) {
      fprintf(stderr, "wrong data ..%s\n", data);
      return false;
    }
    poke_data[i+1] = value;
    fprintf(stderr, "%02x ", poke_data[i+1] & 0xff);
  }
  fprintf(stderr, "\n");

  return send_orb(handle, ORB_POKE_EEPROM, poke_data, 1 + data_len);
}

void iterate_busses(int verbose, int match_vendor, int match_product) {
  /* tbd */
}

static struct usb_dev_handle *open_orb(int verbose, int vendor, int product,
                                       const char *request_version) {
  if (verbose) {
    fprintf(stderr,
            "  dev : vend:prod  |  scan usb busses (vendor / product)\n");
  }
  for (struct usb_bus* bus = usb_busses; bus; bus = bus->next) {
    if (verbose) {
      fprintf(stderr, "/dev/bus/usb/%s/\n", bus->dirname);
    }
    for (struct usb_device* dev = bus->devices; dev; dev = dev->next) {
      const bool is_orb = (dev->descriptor.idVendor == vendor
                          && dev->descriptor.idProduct == product);
      if (verbose) {
        fprintf(stderr, "  %s : %04x:%04x %04x %s\n",
                dev->filename,
                dev->descriptor.idVendor,
                dev->descriptor.idProduct,
                dev->descriptor.bcdDevice,
                is_orb ? "(this is an orb)" : "");
      }
      if (is_orb) {
        usb_dev_handle *handle = usb_open(dev);
        if (handle == NULL)
          continue;
        if (request_version != NULL) {
          if (dev->descriptor.iSerialNumber == 0)
            continue;  // we expect a version, but there is none.
          char device_serial[64];
          usb_get_string_simple(handle, dev->descriptor.iSerialNumber,
                                device_serial,
                                sizeof(device_serial));
          if (strcmp(request_version, device_serial) != 0) {
            usb_close(handle);
            continue;
          }
        }
        return handle;
      }
    }
  }
  return NULL;
}

bool list_available_orb_serials(int verbose, int vendor, int product) {
  int count = 0;
  if (verbose) {
    fprintf(stderr,
            "  dev : vend:prod  |  scan usb busses (vendor / product)\n");
  }
  for (struct usb_bus* bus = usb_busses; bus; bus = bus->next) {
    if (verbose) {
      fprintf(stderr, "bus /dev/bus/usb/%s/\n", bus->dirname);
    }
    for (struct usb_device* dev = bus->devices; dev; dev = dev->next) {
      const int is_orb = (dev->descriptor.idVendor == vendor
                         && dev->descriptor.idProduct == product);
      if (verbose) {
        fprintf(stderr, "  %s : %04x:%04x %04x %s\n",
                dev->filename,
                dev->descriptor.idVendor,
                dev->descriptor.idProduct,
                dev->descriptor.bcdDevice,
                is_orb ? "(this is an orb)" : "");
      }
      if (is_orb && dev->descriptor.iSerialNumber != 0) {
        usb_dev_handle *handle = usb_open(dev);
        if (handle == NULL) {
          continue;
        }
        char device_serial[64];
        memset(device_serial, 0, sizeof(device_serial));
        if (usb_get_string_simple(handle, dev->descriptor.iSerialNumber,
                                  device_serial,
                                  sizeof(device_serial)) >= 0) {
          struct capabilities_t cap;
          if (!get_capabilities(handle, &cap))
            return false;
          fprintf(stdout, "%s\tmax-sequence=%d\thas-capability=[",
                  device_serial, cap.max_sequence_len);
          const char *comma = "";
          if (cap.flags & HAS_GET_COLOR) {
            fprintf(stdout, "%sget-current-color", comma);
            comma = ",";
          }
          if (cap.flags & HAS_GET_SEQUENCE) {
            fprintf(stdout, "%sget-sequence", comma);
            comma = ",";
          }
          if (cap.flags & HAS_AUX) {
            fprintf(stdout, "%saux", comma);
            comma = ",";
          }
          if (cap.flags & HAS_GAMMA_CORRECT) {
            fprintf(stdout, "%sgamma-correction", comma);
            comma = ",";
          }
          if (cap.flags & HAS_CURRENT_LIMIT) {
            fprintf(stdout, "%scurrent-limit", comma);
            comma = ",";
          }
          fprintf(stdout, "]");
          fprintf(stdout, "\tversion=%d\n", cap.version);
          ++count;
        } else {
          print_usb_error(errno);
          return false;
        }
        usb_close(handle);
      }
    }
  }
  if (count == 0) {
    fprintf(stderr, "No orbs found.\n");
    return false;
  }
  return true;
}


// Parse color and duration spec and return number of parameters found.
int parse_color_spec(const char *str, struct rgb_t *out,
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
  out->r = r;
  out->g = g;
  out->b = b;
  if (params >= 4 && morphtime) *morphtime = m;
  if (params >= 5 && duration) *duration = d;
  return params;
}

bool set_sequence(usb_dev_handle *handle, const struct sequence_t *sequence) {
  const int data_len = (sizeof(sequence->count)
                        + sequence->count * sizeof(struct color_period_t));
  return send_orb(handle, ORB_SETCOLOR, sequence, data_len);
}

bool get_sequence(usb_dev_handle *handle, struct sequence_t *sequence) {
  return receive_orb(handle, ORB_GETSEQUENCE, sequence, sizeof(*sequence));
}

bool set_color(usb_dev_handle *handle, struct rgb_t *colors) {
  sequence_t data;
  data.count = 1;
  data.period[0].rgb = *colors;
  data.period[0].morph_time = 0;
  data.period[0].hold_time = 1;
  return set_sequence(handle, &data);
}

static int usage(const char *prog) {
  fprintf(stderr, "usage: %s [options] <RGB-Hex> [<RGB-Hex> ...]\n"
          "Just supply a list of hex colors in the form 'ff0000'\n"
          "\n\thttp://go/microorb/manual\n\n"
          "Options:\n"
          " -s <orb-serial>   : Address an MicroOrb with a particular"
          " serial#.\n"
          "                     Only necessary if there are multiple Orbs"
          " attached.\n"
          " -l                : List all attached Orbs with serial# and"
          " capabilities.\n"
          " -g                : Get current color if supported.\n"
          " -G                : Get current sequence if supported.\n"
          " -v                : Verbose.\n"
          " -x <1|on|0|off>   : Switch aux on/off\n",
          prog);
  return -1;
}

static void format_sequence(const struct sequence_t *seq) {
  int i;
  for (i = 0; i < seq->count; ++i) {
    const struct color_period_t *period = &seq->period[i];
    fprintf(stderr, "%02x%02x%02x:%d:%d ",
            period->rgb.r, period->rgb.g, period->rgb.b,
            period->morph_time * 250, period->hold_time * 250);
  }
  fprintf(stderr, "\n");
}

int main(int argc, char **argv) {
  usb_init();
  usb_find_busses();
  usb_find_devices();

  enum Mode {
    SET_COLOR,
    GET_COLOR,
    GET_SEQUENCE,
    SET_AUX,
    POKE_EEPROM,
    LIST_DEVICES,
  } mode = SET_COLOR;

  int verbose = 0;
  int aux_value = 0;
  char *request_serial = NULL;
  char poke_data[256];

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

      case 'P':
        strncpy(poke_data, optarg, sizeof(poke_data));
        mode = POKE_EEPROM;
        break;

      case 'x':
        mode = SET_AUX;
        if (strcmp(optarg, "on") == 0) {
          aux_value = 1;
        } else if (strcmp(optarg, "off") == 0) {
          aux_value = 0;
        } else {
          char *err;
          aux_value = strtol(optarg, &err, 10);
          if (*err) {
            fprintf(stderr, "invalid integer: '%s'\n", err);
            return usage(argv[0]);
          }
          if ((aux_value & 0x01) != aux_value) {
            fprintf(stderr, "right now, only values 0 or 1\n");
            return usage(argv[0]);
          }
        }
        break;

      default:
        return usage(argv[0]);
    }
  }

  if (mode == LIST_DEVICES) {
    return (list_available_orb_serials(verbose,
                                       ORB_VENDOR, ORB_PRODUCT) ? 0 : 1);
  }

  /* The other commands require to open the partiulcar orb */
  struct usb_dev_handle *handle = open_orb(verbose, ORB_VENDOR, ORB_PRODUCT,
                                           request_serial);
  if (handle == NULL) {
    if (request_serial) {
      fprintf(stderr, "No orb with serial '%s' found.\n", request_serial);
    } else {
      fprintf(stderr, "No orb found.\n");
    }
    return 1;
  }

  if (mode == GET_COLOR) {
    return print_orb_color(handle) ? 0 : 1;
  } else if (mode == GET_SEQUENCE) {
    struct sequence_t seq;
    if (!get_sequence(handle, &seq)) {
      return 1;
    }
    format_sequence(&seq);
  } else if (mode == SET_AUX) {
    return set_aux(handle, aux_value) ? 0 : 1;
  } else if (mode == POKE_EEPROM) {
    return poke_eeprom(handle, poke_data) ? 0 : 1;
  } else {
    struct rgb_t col1;
    const int kArgOffset = optind;

    // Number of colors we got.
    int color_num = argc - kArgOffset;

    if (color_num == 0) {
      return usage(argv[0]);
    }

    if (parse_color_spec(argv[ kArgOffset ], &col1, NULL, NULL) < 3) {
      return usage(argv[0]);
    }

    if (color_num == 1) {  // only a single color.
      set_color(handle, &col1);
    } else {
      if (color_num > kMaxSequenceLen) {
        color_num = kMaxSequenceLen;
        fprintf(stderr, "Exceeded supported number of colors in sequence."
                " Limiting to %d\n", color_num);
      }
      struct sequence_t seq;
      seq.count = color_num;
      int i;
      int morph, duration;
      if (verbose) fprintf(stderr, "Setting %d colors ", color_num);
      for (i = 0; i < color_num; ++i) {
        morph = 0;
        duration = 200;
        if (parse_color_spec(argv[i + kArgOffset],
                             &seq.period[i].rgb, &morph, &duration) < 3) {
          return usage(argv[0]);
        }
        morph = (morph + 249) / 250;
        duration = (duration + 249) / 250;
        seq.period[i].morph_time = morph < 255 ? morph : 255;
        seq.period[i].hold_time = duration < 255 ? duration : 255;
      }
      if (verbose) {
        format_sequence(&seq);
      }
      set_sequence(handle, &seq);
    }
    usb_close(handle);
  }
  return 0;
}
