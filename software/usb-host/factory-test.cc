// Init microorbs with a serial-number.
//
#include <string>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "microorb.h"

#define STATE_FILE "state.txt"
using namespace std;

static struct orb_sequence_t test_sequence = {
    5,
    {
        { { 0x0a, 0x00, 0x00 }, 0, 2 },
        { { 0x0a, 0x00, 0x00 }, 0, 3 },
        { { 0x00, 0x0a, 0x00 }, 0, 3 },
        { { 0x00, 0x00, 0x0a }, 0, 3 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
    }
};

class StateFile {
public:
  StateFile(const string& filename,
	    const string& prefix)
    : filename_(filename), prefix_(prefix),
      letter_('a'), number_(0) {
    Init();
  }

  string GetNextNumber() {
    char buffer[32];
    sprintf(buffer, "MTV%c%03d", letter_, number_);
    ++number_;
    return buffer;
  }

  void Store() {
    FILE* file = fopen(filename_.c_str(), "w");
    if (!file) {
      fprintf(stderr, "Trouble storing file.");
      exit(1);
    }
    fprintf(file, "%c %d\n", letter_, number_);
    fclose(file);
  }

private:
  void Init() {
    FILE* file = fopen(filename_.c_str(), "r");
    if (file) {
      char letter;
      int number;
      if (2 == fscanf(file, "%c %d", &letter, &number)) {
	letter_ = letter;
	number_ = number;
	fprintf(stderr, "Initialized with %c%d\n", letter_, number_);
      } else {
	fprintf(stderr, "Init: not initialized from %s", filename_.c_str());
      }
      fclose(file);
    } else {
      fprintf(stderr, "Couldn't find file %s", filename_.c_str());
    }
  }

  const string filename_;
  const string prefix_;
  char letter_;
  int number_;
};

using orb_driver::MicroOrb;

int main(int argc, char **argv) {
  StateFile store("state.txt", "MTV");
  for (;;) {
    // Waiting for usb connection
    MicroOrb::DeviceList list;
    for (;;) {
      list.clear();
      MicroOrb::UsbList(&list);
      if (list.size() == 1) {
	break;
      } else {
	usleep(100 * 1000);
      }
    }
    // Set testing sequence
    MicroOrb* orb = MicroOrb::Open(list[0]);
    if (orb == NULL) continue;
    fprintf(stderr, "1 ");

    if (!orb->SetSequence(test_sequence)) {
      fprintf(stderr, "ERR");
      delete orb;
      continue;
    }
    fprintf(stderr, "- 2");

    // Writing serial number.
    const string serial = store.GetNextNumber();
    string utf16_serial;
    for (unsigned int i = 0; i < serial.size(); ++i) {
      utf16_serial.append(1, serial[i]);
      utf16_serial.append(1, '\0');
    }
    store.Store();
    // serial is stored at 2
    if (orb->PokeEeprom(2, utf16_serial.data(), utf16_serial.size())) {
      fprintf(stderr, "- 3");
    } else {
      fprintf(stderr, "- ERR");
    }
    delete orb;

    usleep(2500 * 1000);
    fprintf(stderr, "...[unplug]");

    // Wait for disconnect.
    for (;;) {
      list.clear();
      MicroOrb::UsbList(&list);
      if (list.size() == 0) {
	break;
      } else {
	usleep(100 * 1000);
      }
    }
    fprintf(stderr, " OK (%s)\n", serial.c_str());
  }
}
