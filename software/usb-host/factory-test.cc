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

static const int kSerialNumberLen = 7;

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

class PersistentSerialGenerator {
public:
  PersistentSerialGenerator(const string& filename,
                            const string& prefix)
    : filename_(filename), prefix_(prefix),
      current_serial_(0) {
    Init();
    Store();
  }

  const string& GetPrefix() const { return prefix_; }

  string GenerateNextNumber() {
    char buffer[kSerialNumberLen + 1];
    sprintf(buffer, "%s%0*d", prefix_.c_str(), number_len_, current_serial_);
    current_serial_ = (current_serial_ + 1) % max_serial_;
    return buffer;
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
      int number;
      if (2 == fscanf(file, "%s %d", prefix, &number)) {
        if (prefix_ != prefix) {
          fprintf(stderr, "WARNING, prefix in file '%s' overrides '%s'\n",
                  prefix, prefix_.c_str());
        }
        prefix_ = prefix;
	current_serial_ = number;
      } else {
	fprintf(stderr, "Init: not initialized from %s", filename_.c_str());
      }
      fclose(file);
    } else {
      fprintf(stderr, "New file %s", filename_.c_str());
    }
    number_len_ = kSerialNumberLen - prefix_.length();
    max_serial_ = exp(number_len_ * log(10));
    fprintf(stderr, "Initialized with %s%0*d; number-rollover at %d\n",
            prefix_.c_str(), number_len_, current_serial_, max_serial_ - 1);
  }

  const string filename_;
  string prefix_;
  int number_len_;
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

  // This USB implementation is a bit flaky. Send a smaller sequence.
  test_sequence.count = ORB_MAX_SEQUENCE / 4;
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

void DisplaySuccessMessage(bool success, int column) {
  assume_default_colors(COLOR_BLACK, success ? COLOR_GREEN : COLOR_RED);
  const char** banner = success ? ok_banner : fail_banner;
  const int banner_lines = 6;
  int y, dummy;
  getyx(stdscr, y, dummy);
  const int start_y = (LINES - y - banner_lines) / 2 + y;
  for (int i = 0; i < banner_lines; ++i) {
    mvprintw(start_y + i + 1, column, banner[i]);
  }
  refresh();
}

// TODO: implement external photo-sensor.
bool IsDark() {
  return true;
}
bool IsBright() {
  return true;
}

bool SetColorAndSettle(MicroOrb *orb, const orb_rgb_t& color) {
  if (!orb->SetColor(color)) {
    return false;
  }
  orb_rgb_t compare_color;
  for (int i = 0; i < 5; ++i) {
    usleep(200 * 1000);  // let things settle.
    orb->GetColor(&compare_color);
    if (memcmp(&color, &compare_color, sizeof(color)) == 0)
      break;
  }
  if (memcmp(&color, &compare_color, sizeof(color)) != 0)
    return false;
  return true;
}

bool TestSingleColor(MicroOrb *orb, const orb_rgb_t& color, int column) {
  static const orb_rgb_t orb_black = { 0, 0, 0 };
  int y, dummy;
  getyx(stdscr, y, dummy);
  mvprintw(y, column, "- LED color r=%02X g=%02X b=%02X",
           color.red, color.green, color.blue);
  if (!SetColorAndSettle(orb, color)) {
    printw(" setting:FAIL\n");
    return false;
  }
  if (!IsBright()) {
    printw(" seeing:FAIL\n");
    return false;
  }
  SetColorAndSettle(orb, orb_black);
  if (!IsDark()) {
    printw(" set-back-to-black:FAIL\n");
    return false;
  }
  printw(" OK \n");
  refresh();
  return true;
}

bool TestRGBColors(MicroOrb *orb, int column) {
  int y, dummy;
  getyx(stdscr, y, dummy);
  mvprintw(y, column, "* Testing basic RGB function.\n");
  bool success = true;
  const orb_rgb_t red = { 0xff, 0, 0 };
  success &= TestSingleColor(orb, red, column + 2);

  const orb_rgb_t green = { 0, 0xff, 0};
  success &= TestSingleColor(orb, green, column + 2);

  const orb_rgb_t blue = { 0, 0, 0xff};
  success &= TestSingleColor(orb, blue, column + 2);

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

void RunTestLoop(const string &filename, const string &commandline_prefix) {
  PersistentSerialGenerator generator(filename, commandline_prefix);
  bool force_serial = false;

  const int print_column = 16;

  for (;;) {
    erase();
    assume_default_colors(COLOR_WHITE,COLOR_BLACK);
    mvprintw(LINES / 2, COLS / 2 - 10, "[  Waiting for next Orb.  ]");
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
    if (success) success &= TestRGBColors(orb, print_column);
    if (success) success &= WriteSerial(orb, force_serial, print_column,
                                        &generator);

    DisplaySuccessMessage(success, print_column);
    WaitForOrbDisconnect();
  }
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s <filename> <serial-prefix>\n", argv[0]);
    return 1;
  }
  initscr();
  start_color();
  RunTestLoop(argv[1], argv[2]);
  endwin();
  return 0;
}
