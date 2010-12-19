// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)

#include "microorb.h"

#include <string.h>
#include <assert.h>
#include <algorithm>

#include <usb.h>

static const int kMaxSequenceLen = 16;  // Max number of colors sent to Orb.
static const int kUsbTimeoutMs = 1500;

// The timing in the orb firmware is tight and we seem to service the USB
// interrupts not always in time - which leads to broken communication on the
// bus sometimes if we send long sequences.
// So allow for some retries in case of failures.
static const int kUsbRetries = 25;      // Retries in case of usb bus error.

// Memory offset of the current limiting byte in the EEPROM of newer orbs.
static const int kCurrentLimitEepromOffset = 16;
static const int kInitialSequenceEepromOffset = 17;

// The Vendor-ID is usually assigned by some central USB committee for
// cash; We just use the free-to-use 'Prototype product Vendor ID'
static const int kUsbOrbVendor  = 0x6666;
static const int kUsbOrbProduct = 0xF00D;  // Thinking of lunch already ?

namespace orb_driver {

void MicroOrb::UsbList(DeviceList *result) {
  usb_init();
  usb_find_busses();
  usb_find_devices();

  for (struct usb_bus *bus = usb_busses; bus; bus = bus->next) {
    for (struct usb_device *dev = bus->devices; dev; dev = dev->next) {
      const bool is_orb = (dev->descriptor.idVendor == kUsbOrbVendor
                           && dev->descriptor.idProduct == kUsbOrbProduct);
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
  // First handsoldered orbs (internally known as Orb3) have 0x0103, the
  // SMT manufactured Orb4 have 0x0104 in their device version number.
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

const std::string& MicroOrb::GetSerial() {
  if (!serial_.empty())
    return serial_;
  if (device_->descriptor.iSerialNumber == 0)
    return serial_;  // No version available in descriptor; return empty.

  char device_serial[64];
  if (usb_get_string_simple(handle_, device_->descriptor.iSerialNumber,
                            device_serial,
                            sizeof(device_serial)) > 0) {
    serial_ = device_serial;
  }
  return serial_;
}

static void AddCommaString(const std::string& msg, std::string *out) {
  if (!out->empty()) out->append(",");
  out->append(msg);
}

std::string MicroOrb::FormatCapabilitiesString(
    const struct orb_capabilities_t &c) {
  std::string out;
  if (c.flags & HAS_GET_COLOR)     AddCommaString("get-current-color", &out);
  if (c.flags & HAS_GET_SEQUENCE)  AddCommaString("get-sequence", &out);
  if (c.flags & HAS_AUX)           AddCommaString("aux", &out);
  if (c.flags & HAS_GAMMA_CORRECT) AddCommaString("gamma-correction", &out);
  if (c.flags & HAS_CURRENT_LIMIT) AddCommaString("current-limit", &out);

  return out;
}

// Older orbs don't support the current limiting in firmware. So do it here in
// case we'd reach the 500mA: limit by scaling the individual colors.
void MicroOrb::LEDCurrentLimit(struct orb_sequence_t *seq) {
  if (IsOrb4()) return;  // we're good.

  // We want at most 500mA. Each LED takes empirically around 280mA. The current
  // is mapped on a range of 0..255.
  const float max_value = 500.0 / 280.0 * 255.0;
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

// Checks if the sequence a and b are the same.
static bool SequenceEqual(const struct orb_sequence_t &a,
                          const struct orb_sequence_t &b) {
  if (a.count != b.count) return false;
  for (int i = 0; i < a.count; ++i) {
    const struct orb_color_period_t &p1 = a.period[i];
    const struct orb_color_period_t &p2 = b.period[i];
    if (p1.morph_time != p2.morph_time
        || p1.hold_time != p2.hold_time
        || p1.color.red != p2.color.red
        || p1.color.green != p2.color.green
        || p1.color.blue != p2.color.blue)
      return false;
  }
  return true;
}

bool MicroOrb::SetSequence(const struct orb_sequence_t &sequence) {
  // Don't overwhelm older orbs with long color sequences.
  const int real_count = IsOrb4() ? sequence.count : 1;
  const int data_len = (sizeof(sequence.count)
                        + real_count * sizeof(struct orb_color_period_t));

  // If we have an old orb, we need to take care of some current limiting.
  struct orb_sequence_t current_limited = sequence;
  LEDCurrentLimit(&current_limited);

  for (int i = 0; i < kUsbRetries; ++i) {
    if (!Send(ORB_SETSEQUENCE, &current_limited, data_len))
      return false;

    // Unfortunately, sometimes things don't work out properly due to timing
    // issues and data gets garbled especially with longer sequences.
    // Retrieve the current sequence and verify that it is indeed the same we
    // sent.

    if (!IsOrb4())
      return true;   // Old orbs don't support GetSequence()

    struct orb_sequence_t verify;
    GetSequence(&verify);
    if (SequenceEqual(current_limited, verify))
      return true;
  }
  return false;
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
    int data_len = std::min(end_pos - pos, kChunkSize);
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
  return PokeEeprom(kCurrentLimitEepromOffset, &to_send, 1);
}

bool MicroOrb::SetInitialSequence(const struct orb_sequence_t &sequence) {
  if (!IsOrb4()) return false;
  const int data_len = (sizeof(sequence.count)
                        + sequence.count * sizeof(struct orb_color_period_t));
  return PokeEeprom(kInitialSequenceEepromOffset, &sequence, data_len);
}
}  // end namespace orb_driver
