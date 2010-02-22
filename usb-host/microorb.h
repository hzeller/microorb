// Copyright 2008 Google Inc. All Rights Reserved.
// Author: hzeller@google.com (Henner Zeller)
//
// GPL license.

#include <string>
#include <vector>
#include "../microorb-protocol.h"

using namespace std;  // non-google3 compilation.

// in <usb.h>
struct usb_device;
struct usb_dev_handle;

class MicroOrb {
 public:
  ~MicroOrb();

  // Get a list of Microobs on our USB busses.
  typedef vector<struct usb_device*> DeviceList;
  static void UsbList(DeviceList *result);

  // Open a MicroOrb from an usb device or NULL if not possible for some reason.
  static MicroOrb* Open(struct usb_device *device);

  // Get the serial number of this orb or an empty string if for some reason
  // not retrievable.
  const string& GetSerial();

  // Get capabilities.
  bool GetCapabilities(struct orb_capabilities_t *capabilities);

  // Utility: format a string with the capabilities of this Orb.
  static string FormatCapabilitiesString(const struct orb_capabilities_t &cap);

  // Set the color of the orb.
  bool SetColor(const struct orb_rgb_t &color);

  // Get current color of orb.
  bool GetColor(struct orb_rgb_t *color);

  // Set color sequence.
  bool SetSequence(const struct orb_sequence_t &sequence);

  // Get a color sequence.
  bool GetSequence(struct orb_sequence_t *sequence);

  // Set the aux pin on the orb if supported.
  bool SetAux(bool value);

  // -- Features of Orb 4

  // Poke data into the orb's eeprom if supported. Do only if you know
  // what'ya doing.
  bool PokeEeprom(int offset, const void *buffer, int len);

  // Switch off current limiting if supported. That will operate the orb
  // outside the USB specification; dangerous on USB hubs that don't support
  // that.
  bool SwitchCurrentLimit(bool value);

  // Set the startup sequence for this Orb.
  bool SetInitialSequence(const struct orb_sequence_t &sequence);

 private:
  MicroOrb(struct usb_device *device, struct usb_dev_handle *handle)
    : device_(device), handle_(handle) {}

  bool Send(enum OrbRequest command, const void* input, size_t data_len);
  bool Receive(enum OrbRequest command, void* buffer, size_t buffer_len);

  // Returns if this is an orb4 device with some more features. It is cheaper
  // to call this function than the getting the capabilities.
  bool IsOrb4() const;

  const struct usb_device *const device_;
  struct usb_dev_handle *const handle_;
  string serial_;
};
