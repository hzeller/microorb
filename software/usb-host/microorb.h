// Copyright (c) 2008 Henner Zeller <h.zeller@acm.org>
// This software is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2, as published
// by the Free Software Foundation <http://www.gnu.org/copyleft/>.

#ifndef TESTING_ORB_DRIVERS_MICROORB_MICROORB_H_
#define TESTING_ORB_DRIVERS_MICROORB_MICROORB_H_

#include "../microorb-protocol.h"

#include <string>
#include <vector>

// from usb.h
struct usb_device;
struct usb_dev_handle;

namespace orb_driver {

class MicroOrb {
 public:

  ~MicroOrb();

  // Get a list of Microobs on our USB busses.
  typedef std::vector<struct usb_device*> DeviceList;
  static void UsbList(DeviceList *result);

  // Create a MicroOrb from an usb device or NULL on failure.
  // The usb_device should be retrieved using the UsbList() function.
  static MicroOrb* Open(struct usb_device *device);

  // Get the serial number of this orb or an empty string if for some reason
  // not retrievable.
  // Each orb has a unique serial number (something like "MTV0042") so that it
  // is possible to address a specific one if multiple are connected.
  const std::string& GetSerial();

  // Set the serial number with exactly 7 characters and store in Orb.
  // Returns the success of writing the serial number to EEPROM.
  // Only for Orb versions > 4
  bool SetSerial(const std::string& serial);

  // Get capabilities.
  bool GetCapabilities(struct orb_capabilities_t *capabilities);

  // Utility: format a string with the capabilities of this Orb.
  static std::string FormatCapabilitiesString(
      const struct orb_capabilities_t &cap);

  // Set the color of the orb.
  bool SetColor(const struct orb_rgb_t &color);

  // Get current color of orb. If the orb is just morphing between two colors,
  // then this will the current intermediate color.
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
  // that. This setting is stored in EEPROM, thus survives a 'reboot' of Orb.
  bool SwitchCurrentLimit(bool value);

  // Set the startup sequence for this Orb. This is stored in EEPROM.
  bool SetInitialSequence(const struct orb_sequence_t &sequence);

 private:
  MicroOrb(struct usb_device *device, struct usb_dev_handle *handle)
      : device_(device), handle_(handle) {}

  bool Send(enum OrbRequest command, const void* input, size_t data_len);
  bool Receive(enum OrbRequest command, void* buffer, size_t buffer_len);

  // Returns if this is an orb4 device with some more features. It is cheaper
  // to call this function than the getting the capabilities.
  bool IsOrb4() const;

  // Some legacy; older orbs don't do current limiting, so it could be that
  // colors that use multiple LEDs can suck too much current. Fix it here.
  void LEDCurrentLimit(struct orb_sequence_t *sequence);

  const struct usb_device *const device_;  // not owned.
  struct usb_dev_handle *const handle_;  // allocated by usb_open()/usb_close()
  std::string serial_;
};

}  // end namespace orb_driver

#endif  // TESTING_ORB_DRIVERS_MICROORB_MICROORB_H_
