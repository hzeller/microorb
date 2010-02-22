// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// GPL license.

#include "microorb.h"

#include <string.h>
#include <usb.h>
#include <assert.h>

static const int kMaxSequenceLen = 16;  // Max number of colors sent to Orb.
static const int kUsbTimeoutMs = 1500;
static const int kUsbRetries = 3;       // Retries in case of usb bus error.

/* we use as vendor 'Prototype product Vendor ID' */
#define ORB_VENDOR  0x6666
#define ORB_PRODUCT 0xF00D

void MicroOrb::UsbList(DeviceList *result) {
  usb_init();
  usb_find_busses();
  usb_find_devices();

  for (struct usb_bus* bus = usb_busses; bus; bus = bus->next) {
    for (struct usb_device* dev = bus->devices; dev; dev = dev->next) {
      const bool is_orb = (dev->descriptor.idVendor == ORB_VENDOR
                           && dev->descriptor.idProduct == ORB_PRODUCT);
      if (is_orb) {
        result->push_back(dev);
      }
    }
  }
}

MicroOrb* MicroOrb::Open(struct usb_device *device) {
  usb_dev_handle *handle = NULL;
  for (int i = 0; handle == NULL && i < kUsbRetries; ++i)
    handle = usb_open(device);
  if (handle == NULL)
    return NULL;
  return new MicroOrb(device, handle);
}

MicroOrb::~MicroOrb() {
  usb_close(handle_);
}

bool MicroOrb::IsOrb4() const {
  return device_->descriptor.bcdDevice == 0x0104;
}

bool MicroOrb::Send(enum OrbRequest command,
                    const void* input, size_t data_len) {
  int result = -1;
  char *const data = const_cast<char*>(reinterpret_cast<const char*>(input));
  for (int i = 0; result < 0 && i < kUsbRetries; ++i) {
    result = usb_control_msg(handle_,
                             USB_ENDPOINT_OUT
                             | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                             command, 0, 0,
                             data, data_len,
                             kUsbTimeoutMs);
  }
  return result >= 0;
}

// Receive data from orb and store it in given buffer. Buffer is reset.
bool MicroOrb::Receive(enum OrbRequest command,
                       void* buffer, size_t buffer_size) {
  int result = -1;
  for (int i = 0; result < 0 && i < kUsbRetries; ++i) {
    memset(buffer, 0, buffer_size);
    result = usb_control_msg(handle_,
                             USB_ENDPOINT_IN
                             | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
                             command, 0, 0,
                             reinterpret_cast<char*>(buffer), buffer_size,
                             kUsbTimeoutMs);
  }
  return result >= 0;
}

const string& MicroOrb::GetSerial() {
  if (!serial_.empty())
    return serial_;
  if (device_->descriptor.iSerialNumber == 0)
    return serial_;  // we expect a version, but there is none.

  char device_serial[64];
  if (usb_get_string_simple(handle_, device_->descriptor.iSerialNumber,
                            device_serial,
                            sizeof(device_serial)) > 0) {
    serial_ = device_serial;
  }
  return serial_;
}

static void AddCommaString(const string& msg, string *out) {
  if (!out->empty()) out->append(",");
  out->append(msg);
}

string MicroOrb::FormatCapabilitiesString(const struct orb_capabilities_t &c) {
  string out;
  if (c.flags & HAS_GET_COLOR)     AddCommaString("get-current-color", &out);
  if (c.flags & HAS_GET_SEQUENCE)  AddCommaString("get-sequence", &out);
  if (c.flags & HAS_AUX)           AddCommaString("aux", &out);
  if (c.flags & HAS_GAMMA_CORRECT) AddCommaString("gamma-correction", &out);
  if (c.flags & HAS_CURRENT_LIMIT) AddCommaString("current-limit", &out);

  return out;
}

void MicroOrb::LEDCurrentLimit(struct orb_sequence_t *seq) {
  if (IsOrb4()) return;  // we're good.

  // We want at most 500mA. Each LED takes empirically around 280mA.
  const float max_value = 500.0/280.0 * 255.0;
  for (int i = 0; i < seq->count; ++i) {
    int total_current = (seq->period[i].color.red
                         + seq->period[i].color.green
                         + seq->period[i].color.blue);
    if (total_current > max_value) {
      const float factor = max_value / total_current;
      seq->period[i].color.red *= factor;
      seq->period[i].color.green *= factor;
      seq->period[i].color.blue *= factor;
    }
  }
}
bool MicroOrb::SetSequence(const struct orb_sequence_t &sequence) {
  // Don't overwhelm older orbs with long color sequence.
  const int real_count = IsOrb4() ? sequence.count : 1;
  const int data_len = (sizeof(sequence.count)
                        + real_count * sizeof(struct orb_color_period_t));
  // If we have an old orb, we need to take care of some current limiting.
  struct orb_sequence_t current_limited = sequence;
  LEDCurrentLimit(&current_limited);
  return Send(ORB_SETSEQUENCE, &current_limited, data_len);
}

bool MicroOrb::GetSequence(struct orb_sequence_t *sequence) {
  if (!IsOrb4()) return false;
  return Receive(ORB_GETSEQUENCE, sequence, sizeof(*sequence));
}

bool MicroOrb::GetCapabilities(struct orb_capabilities_t *capabilities) {
  return Receive(ORB_GETCAPABILITIES, capabilities, sizeof(*capabilities));
}

bool MicroOrb::SetColor(const struct orb_rgb_t &color) {
  orb_sequence_t data;
  data.count = 1;
  data.period[0].color = color;
  data.period[0].morph_time = 0;
  data.period[0].hold_time = 1;
  return SetSequence(data);
}

bool MicroOrb::GetColor(struct orb_rgb_t *color) {
  return Receive(ORB_GETCOLOR, color, sizeof(*color));
}

bool MicroOrb::SetAux(bool value) {
  char to_send = value ? 1 : 0;
  return Send(ORB_SETAUX, &to_send, sizeof(to_send));
}

bool MicroOrb::PokeEeprom(int eeprom_offset, const void *buffer, int len) {
  if (!IsOrb4()) return false;
  const int end_pos = eeprom_offset + len;
  if (eeprom_offset < 0 || len < 0 || end_pos > 128) return false;
  // we only can write in chunks of 7
  const int kChunkSize = 7;
  for (int pos = eeprom_offset; pos < end_pos; pos += kChunkSize) {
    int data_len = min(end_pos - pos, kChunkSize);
    char poke_data[kChunkSize + 1];
    poke_data[0] = pos;
    const char *char_buf = reinterpret_cast<const char*>(buffer);
    memcpy(poke_data + 1, char_buf + pos - eeprom_offset, data_len);
    if (!Send(ORB_POKE_EEPROM, poke_data, 1 + data_len))
      return false;
  }
  return true;
}

bool MicroOrb::SwitchCurrentLimit(bool value) {
  if (!IsOrb4()) return false;
  char to_send = ORB_SWITCH_CURRENT_LIMIT_OFF_MAGIC;
  if (value)
    to_send = ~to_send;
  return PokeEeprom(16, &to_send, 1);
}
