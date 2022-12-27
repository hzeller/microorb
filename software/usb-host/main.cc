// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (c) 2008 Henner Zeller <h.zeller@acm.org>
// This software is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2, as published
// by the Free Software Foundation <http://www.gnu.org/copyleft/>.
//
// Talk to the microorb. Commandline interface to class MicroOrb.
//
// This requires libusb (tested on Linux and MacOS)

#include <arpa/inet.h>
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <map>
#include <string>

#include <microhttpd.h>

#include "microorb.h"

using orb_driver::MicroOrb;
using std::string;
using std::map;

// Parameters we pass to the daemon.
struct HttpServingParameters {
  MicroOrb *orb;
  const char *requested_serial;
  bool verbose;
};

static int usage(const char *prog) {
  fprintf(stderr,
          "usage: %s [options] <RGB-Hex>[:<morph-ms>[:<hold-ms>]] [<RGB>...]\n"
          "Just supply a list of one or more hex colors like: FF0000\n"
          ".. or with transition times: FF0000:500:3000 00FF00:500:3000\n\n"
          "Options:\n"
          " -s <orb-serial>   : Address a MicroOrb with a particular"
          " serial#.\n"
          "                     Only necessary if there are multiple Orbs"
          " attached.\n"
          " -l                : List all attached Orbs with serial# and"
          " capabilities.\n"
          " -g                : Get current color.\n"
          " -G                : Get current sequence if supported.\n"
          " -P <port>         : HTTP-service on port (experimental).\n"
          "                       -v prints HTTP access log.\n"
          " -S <sequence>     : Replace startup sequence with given.\n"
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

int AcceptColorParam (void *cls, enum MHD_ValueKind kind,
                      const char *key, const char *value)
{
  if (strcmp(key, "c") == 0) {
    string* ret_value = (string*) cls;
    *ret_value = value;
  }
  return MHD_YES;
}

bool GetResource(const string &name, int *size, const char **data) {
  typedef map<string, string*> ResourceMap;
  static ResourceMap resources;
  string resource_name = (name == "/") ? "/index.html" : name;
  ResourceMap::const_iterator found = resources.find(resource_name);
  string *result;
  if (found != resources.end()) {
    result = found->second;
  } else {
    const int fd = open(("web-resource/" + resource_name).c_str(), O_RDONLY);
    if (fd < 0) return false;
    result = new string();
    char buffer[8192];
    int r;
    while ((r = read(fd, buffer, sizeof(buffer))) > 0) {
      result->append(buffer, r);
    }
    close(fd);
    // We only have a handful resources. Cache into and serve from memory.
    resources[resource_name] = result;
  }

  *data = result->data();
  *size = result->length();
  return true;
}

// MicroHTTP chops up the URI to parameters. However, we want to log them
// so save a copy beforehand.
static void *HttpSaveUri(void *user_argument, const char *uri) {
  HttpServingParameters *params = (HttpServingParameters*) user_argument;
  return params->verbose ? strdup(uri) : NULL;
}

void *extract_addr(const struct sockaddr *sa) {
  return (sa->sa_family == AF_INET)
    ? (void*) &(((struct sockaddr_in*)sa)->sin_addr)
    : (void*) &(((struct sockaddr_in6*)sa)->sin6_addr);
}

static void WriteHttpLog(struct MHD_Connection *connection, const char *method,
                         const char *logging_uri) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  struct tm *tmp = localtime(&tv.tv_sec);
  char time_buf[20];
  strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tmp);
  char ip_buf[INET6_ADDRSTRLEN];
  const struct sockaddr *client
    = MHD_get_connection_info(connection, MHD_CONNECTION_INFO_CLIENT_ADDRESS,
                              MHD_OPTION_END)->client_addr;
  inet_ntop(client->sa_family, extract_addr(client), ip_buf, sizeof(ip_buf));
  fprintf(stderr, "%s.%06ld\t%s\t%s %s\n", time_buf, tv.tv_usec, ip_buf,
          method, logging_uri);
}

#if MHD_VERSION >= 0x00097002
  typedef MHD_Result HandleHttpResult;
#else
  typedef int HandleHttpResult;
#endif

static HandleHttpResult HandleHttp(void* user_argument,
                                   struct MHD_Connection *connection,
                                   const char *url,
                                   const char *method, const char *version,
                                   const char *upload_data, size_t *upload_size,
                                   void** allocated_logging_uri) {
  // Not cool yet; should pre-set to the current color and also should start
  // out showing the color-chooser, not only the input-field.
  // So someone please poke in the web-resource/ directory to make this nice.
  // ( Maybe add a #<current-color> to the URL, so that the index.html can
  //   pick it up ?
  //   Also, right now we only support one color to set; maybe we can hack
  //   the client javascript to provide a 'UI' that allows setting a sequence?
  //   ... lots to do for people liking to hack JavaScript).

  HttpServingParameters *params = (HttpServingParameters*) user_argument;
  string result;
  struct MHD_Response *response;
  HandleHttpResult ret;

  if (params->verbose) {
    assert(allocated_logging_uri);
    WriteHttpLog(connection, method, (const char*) *allocated_logging_uri);
    free(*allocated_logging_uri);
  }

  // Setting color that we get via query-paramter 'c'
  if (strcmp(url, "/set") == 0) {
    struct orb_sequence_t seq;
    const char *args[1];
    const char *color_arg
      = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, "c");
    args[0] = color_arg ? color_arg : "000000";
    if (!ParseSequence(args, 1, &seq)) {
      response = MHD_create_response_from_buffer(4, (void*)"FAIL",
                                                 MHD_RESPMEM_PERSISTENT);
      ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
    } else {
      int attempts = 2;
      while (attempts-- && (!params->orb || !params->orb->SetSequence(seq))) {
        fprintf(stderr, "Lost connection. Reconnect orb.\n");
        delete params->orb;
        params->orb = OpenOrb(params->requested_serial);
      }
      response = MHD_create_response_from_buffer(2, (void*)"OK",
                                                 MHD_RESPMEM_PERSISTENT);
      ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    }
  }
  else {
    // Everything else: serve any static content.
    int size;
    const char *buffer;
    if (GetResource(url, &size, &buffer)) {
      response = MHD_create_response_from_buffer(size, (void*) buffer,
                                                 MHD_RESPMEM_PERSISTENT);
      ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    } else {
      response = MHD_create_response_from_buffer(0, (void*)"",
                                                 MHD_RESPMEM_PERSISTENT);
      ret = MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, response);
    }
  }
  MHD_destroy_response(response);
  return ret;
}

int main(int argc, char **argv) {
  enum Mode {
    SET_COLOR_SEQUENCE,
    GET_COLOR,
    GET_SEQUENCE,
    SET_AUX,
    LIST_DEVICES,
    SET_INITIAL_SEQUENCE,
    HTTP_SERVICE,
  } mode = SET_COLOR_SEQUENCE;

  int verbose = 0;
  char aux_value = 0;
  char *request_serial = NULL;
  int port = -1;
  int opt;
  while ((opt = getopt(argc, argv, "gGvhlSs:x:P:")) != -1) {
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

    case 'S':
      mode = SET_INITIAL_SEQUENCE;
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

    case 'P':
      mode = HTTP_SERVICE;
      port = atoi(optarg);
      break;

    default:
      return usage(argv[0]);
    }
  }

  const int kArgOffset = optind;
  const int arg_num = argc - kArgOffset;

  if ((mode == SET_COLOR_SEQUENCE || mode == SET_INITIAL_SEQUENCE)
      && arg_num == 0) {
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

  if (port > 0 && mode == HTTP_SERVICE) {
    struct MHD_Daemon *daemon;
    struct HttpServingParameters params;
    params.orb = orb;
    params.requested_serial = request_serial;
    params.verbose = verbose;
    daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, port, NULL, NULL,
                              &HandleHttp, &params,
                              MHD_OPTION_URI_LOG_CALLBACK, &HttpSaveUri, &params,
                              MHD_OPTION_END);
    if (NULL == daemon) return 1;
    for (;;) sleep(1000);
    MHD_stop_daemon(daemon);
    return 0;
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
    if (mode == SET_COLOR_SEQUENCE) {
      return orb->SetSequence(seq) ? 0 : 1;
    } else if (mode == SET_INITIAL_SEQUENCE) {
      return orb->SetInitialSequence(seq) ? 0 : 1;
    }
  }
  delete orb;
  return 0;
}
