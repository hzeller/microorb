// Init microorbs with a serial-number.
// Little quick'n dirty program for assembly folks in the field.
//
#include <string>
#include <iostream>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ncurses.h>

#include "microorb.h"

#define STATE_FILE "state.txt"

// Size of sequence to set for sequence set/get test. Since there seem to
// be trouble with the USB synchronization sometimes, we don't go the full
// length of 16 but a bit shorter.
static const int kColorSequenceLength = 16;

static const int kSerialNumberLen = 7;
static const int kMinSerialNumberDigits = 3;

using std::string;

static const char* fail_banner[] = {
  " ######    ##       #    #",
  " #        #  #      #    #",
  " #####   #    #     #    #",
  " #       ######     #    #",
  " #       #    #     #    #",
  " #       #    #     #    ######"
};

static const char* ok_banner[] = {
  "      ####   #    #",
  "     #    #  #   #",
  "     #    #  ####",
  "     #    #  #  #",
  "     #    #  #   #",
  "      ####   #    #"
};

// For human consumption, this should not be too bright.
static const int kRGBTestBrightness = 0x0f;

// A Light-sensor to determine if the LED is doing what we expect.
class LightSensor {
public:
  virtual ~LightSensor() {}
  virtual bool IsBright() = 0;
  virtual bool IsDark() = 0;
};

// This dummy light sensor always returns what we expect. TODO: build real
// external sensor.
class DummyLightSensor : public LightSensor {
public:
  virtual bool IsBright() {
    usleep(kHumanObservationTime * 1000); // slow down things.
    return true;
  }
  virtual bool IsDark() { return true; }
private:
  static const int kHumanObservationTime = 500; /* ms */
};

class PersistentSerialGenerator {
public:
  PersistentSerialGenerator(const string& filename,
                            const string& prefix)
    : filename_(filename), prefix_(prefix),
      current_serial_(0) {
    if (prefix.length() >= kSerialNumberLen) {
      fprintf(stderr, "Prefix too long: '%s' longer than max serial len %d\n",
	      prefix.c_str(), kSerialNumberLen);
      exit(1);
    }
    Init();
    Store();
  }

  const string& GetPrefix() const { return prefix_; }

  string UpcomingNumber() const {
    char buffer[kSerialNumberLen + 1];
    sprintf(buffer, "%s%0*d", prefix_.c_str(), serial_digits_, current_serial_);
    return buffer;
  }

  string GenerateNextNumber() {
    string result = UpcomingNumber();
    current_serial_ = (current_serial_ + 1) % max_serial_;
    return result;
  }

  void Store() {
    FILE* file = fopen(filename_.c_str(), "w");
    if (!file) {
      fprintf(stderr, "Trouble storing state file.");
      exit(1);
    }
    fprintf(file, "%s %d\n", prefix_.c_str(), current_serial_);
    fclose(file);
  }

private:
  void Init() {
    FILE* file = fopen(filename_.c_str(), "r");
    if (file) {
      char prefix[kSerialNumberLen + 1];
      int number = -1;
      if (2 == fscanf(file, "%s %d", prefix, &number)) {
        if (prefix_ != prefix) {
          fprintf(stderr, "Prefix in file '%s' mismatches '%s' on commandline. "
                  "Plese fix.\n", prefix, prefix_.c_str());
          exit(1);
        }
	fprintf(stderr, "hz: strlen(prefix) = %zd ('%s', %d)\n",
		strlen(prefix), prefix, number);
        prefix_ = prefix;
	current_serial_ = number;
      } else {
	fprintf(stderr, "Init: not initialized from %s", filename_.c_str());
      }
      fclose(file);
    } else {
      fprintf(stderr, "New file '%s'\nxs", filename_.c_str());
    }

    serial_digits_ = kSerialNumberLen - prefix_.length();
    if (serial_digits_ < kMinSerialNumberDigits) {
      fprintf(stderr, "Cannot fit at least %d digits in serial number "
	      "if prefix is '%s'\n", kMinSerialNumberDigits, prefix_.c_str());
      exit(1);
    }
    max_serial_ = exp(serial_digits_ * log(10));
    fprintf(stderr, "Initialized with %s%0*d; number-rollover at %d\n",
            prefix_.c_str(), serial_digits_, current_serial_, max_serial_ - 1);
  }

  const string filename_;
  string prefix_;
  int serial_digits_;
  int max_serial_;
  int current_serial_;
};

using orb_driver::MicroOrb;

static bool HasPrefixString(const string& str, const string& prefix) {
  return strncmp(prefix.c_str(), str.c_str(), prefix.length()) == 0;
}

struct usb_device *WaitForOrbConnect() {
  // Waiting for usb connection
  MicroOrb::DeviceList list;
  for (;;) {
    list.clear();
    MicroOrb::UsbList(&list);
    if (list.size() == 1) {
      return list[0];
    } else {
      usleep(100 * 1000);
    }
  }
}

void WaitForOrbDisconnect() {
  // Wait for disconnect.
  MicroOrb::DeviceList list;
  for (;;) {
    list.clear();
    MicroOrb::UsbList(&list);
    if (list.size() == 0) {
      break;
    } else {
      usleep(100 * 1000);
    }
  }
}

bool SetAndReadTestSequence(MicroOrb* orb, int column) {
  // Setting a sequence and re-reading it.
  struct orb_sequence_t test_sequence;
  memset(&test_sequence, 0, sizeof(test_sequence));

  test_sequence.count = kColorSequenceLength;
  for (unsigned char i = 0; i < test_sequence.count; ++i) {
    test_sequence.period[i].morph_time = i;
    test_sequence.period[i].hold_time = 2 * i + ORB_MAX_SEQUENCE;
  }

  int y, dummy;
  getyx(stdscr, y, dummy);
  mvprintw(y, column, "* Testing basic read/write of USB interface");
  refresh();
  mvprintw(y + 1, column + 2, "- Writing test sequence with %d colors.",
           test_sequence.count);
  if (orb->SetSequence(test_sequence)) {
    printw(" OK\n");
  } else {
    printw(" FAIL\n");
    return false;
  }
  refresh();

  mvprintw(y + 2, column + 2, "- Re-reading test sequence.");
  bool success = false;
  orb_sequence_t re_read;
  memset(&re_read, 0, sizeof(re_read));
  for (int i = 0; i < 5; ++i) {
    if (orb->GetSequence(&re_read) && memcmp(&test_sequence, &re_read,
                                             sizeof(test_sequence)) == 0) {
      printw(" OK\n");
      success = true;
      break;
    }
  }
  if (!success) printw(" FAIL\n");
  refresh();
  return success;
}

void DisplayTestResultMessage(bool success, int column) {
  assume_default_colors(COLOR_BLACK, success ? COLOR_GREEN : COLOR_RED);
  const char** banner = success ? ok_banner : fail_banner;
  const int banner_lines = 6;
  int y, dummy;
  getyx(stdscr, y, dummy);
  const int start_y = (LINES - y - banner_lines) / 2 + y;
  for (int i = 0; i < banner_lines; ++i) {
    mvprintw(start_y + i + 1, column, "%s", banner[i]);
  }
  refresh();
}

bool SetColorAndSettle(MicroOrb *orb, const orb_rgb_t& color) {
  if (!orb->SetColor(color)) {
    return false;
  }
  orb_rgb_t compare_color;
  for (int i = 0; i < 5; ++i) {
    usleep(50 * 1000);  // let things settle.
    orb->GetColor(&compare_color);
    if (memcmp(&color, &compare_color, sizeof(color)) == 0)
      break;
  }
  if (memcmp(&color, &compare_color, sizeof(color)) != 0)
    return false;
  return true;
}

bool TestSingleColor(MicroOrb *orb, LightSensor *light_sensor,
                     const orb_rgb_t& color, int column) {
  static const orb_rgb_t orb_black = { 0, 0, 0 };
  int y, dummy;
  getyx(stdscr, y, dummy);
  mvprintw(y, column, "- LED color [%02x %02x %02x]",
           color.red, color.green, color.blue);
  if (!SetColorAndSettle(orb, color)) {
    printw(" setting:FAIL\n");
    return false;
  }
  if (!light_sensor->IsBright()) {
    printw(" seeing:FAIL\n");
    return false;
  }
  SetColorAndSettle(orb, orb_black);
  if (!light_sensor->IsDark()) {
    printw(" set-back-to-black:FAIL\n");
    return false;
  }
  printw(" OK \n");
  refresh();
  return true;
}

bool TestRGBColors(MicroOrb *orb, LightSensor *light_sensor, int column) {
  int y, dummy;
  getyx(stdscr, y, dummy);
  mvprintw(y, column, "* Testing basic RGB function.\n");
  bool success = true;
  const orb_rgb_t red = { kRGBTestBrightness, 0, 0 };
  success &= TestSingleColor(orb, light_sensor, red, column + 2);

  const orb_rgb_t green = { 0, kRGBTestBrightness, 0 };
  success &= TestSingleColor(orb, light_sensor, green, column + 2);

  const orb_rgb_t blue = { 0, 0, kRGBTestBrightness };
  success &= TestSingleColor(orb, light_sensor, blue, column + 2);

  return success;
}

bool WriteSerial(MicroOrb *orb, bool force, int column,
                 PersistentSerialGenerator *generator) {
  int y, dummy;
  getyx(stdscr, y, dummy);
  if (!force && HasPrefixString(orb->GetSerial(), generator->GetPrefix())) {
    mvprintw(y, column, "* Skipping setting serial. Existing serial '%s' "
             "already has prefix '%s'\n", orb->GetSerial().c_str(),
             generator->GetPrefix().c_str());
    return true;
  }
  const string new_serial = generator->GenerateNextNumber();
  generator->Store();
  mvprintw(y, column, "* Writing new serial number '%s' ", new_serial.c_str());
  const bool success = orb->SetSerial(new_serial);
  printw(success ? " OK\n" : " FAIL\n");
  return success;
}

void RunTestLoop(PersistentSerialGenerator *generator) {
  bool force_serial = false;

  const int print_column = 16;
  DummyLightSensor manual_light_sensor;

  for (;;) {
    erase();
    assume_default_colors(COLOR_WHITE,COLOR_BLACK);
    mvprintw(LINES / 2 - 1, COLS / 2 - 13, "[  Waiting for next Orb. ]");
    mvprintw(LINES / 2, COLS / 2 - (17 + kSerialNumberLen) / 2,
	     "Upcoming serial# %s", generator->UpcomingNumber().c_str());
    mvprintw(LINES / 2 + 1, COLS / 2, "%s", "");
    refresh();

    bool success = true;
    MicroOrb* orb = MicroOrb::Open(WaitForOrbConnect());
    if (orb == NULL) continue;
    erase();

    mvprintw(5, print_column, "* Connect\n");
    refresh();

    // suppress initial sequence as soon as possible to not blind people :)
    const orb_rgb_t black = { 0, 0, 0 };
    orb->SetColor(black);

    // Print current serial if any.
    orb->GetSerial();
    usleep(200 * 1000); // Let things settle.
    const string serial = orb->GetSerial();
    mvprintw(6, print_column,
             "* Getting existing serial number '%s'\n", serial.c_str());
    refresh();

    if (success) success &= SetAndReadTestSequence(orb, print_column);
    if (success) success &= WriteSerial(orb, force_serial, print_column,
                                        generator);
    // At this point we know that things work electrically and on the USB bus.
    // Now display the RGB colors for the human tester to verify.
    // We do this at the end: the tester knows that (s)he can safely unplug
    // the Orb afterwards. Also, if the color sequence does not show up, we
    // know something is wrong without looking at the screen.
    if (success) success &= TestRGBColors(orb, &manual_light_sensor,
                                          print_column);

    DisplayTestResultMessage(success, print_column);
    WaitForOrbDisconnect();
  }
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <serial-prefix>\n", argv[0]);
    return 1;
  }
  const string serial_prefix = argv[1];
  const string filename = "serial-" + serial_prefix + ".txt";
  PersistentSerialGenerator generator(filename, serial_prefix);

  initscr();
  start_color();
  RunTestLoop(&generator);
  endwin();
  return 0;
}
