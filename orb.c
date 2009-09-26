/* USB orb with an attiny44
 * (c) 2008 Henner Zeller <h.zeller@acm.org>
 * This softare is GPL licensed.
 *
 * We have 2k flash and 128 bytes RAM (of which are already ~1700 bytes flash 
 * and ~50 bytes RAM used by the USB implementation), but want all these
 * Features
 *  + setting a single color
 *  + switching the AUX port
 *
 * .. Not-yet-implemented features:
 *  - set multiple colors and cycle between them according to the 'hold' time.
 *    (as many colors as memory allows)
 *  - do a smooth morph between adjacend colors in the cycle according to the
 *    'morph' time.
 *  - after a setting has been active for more than 1min, store it in eeprom
 *    and restore it on startup. Only store _changed_ configuration because
 *    eeprom has a limited number of writes.
 *  - store the serial number in eeprom instead of the code.
 *  - do gamma correction by having a mapping from 255 color values to
 *    1024 PWM values; do 1024 PWM. Have the right relationships between
 *    colors so that 'white' is white.
 *  - Do the PWM in an interrupt handler.
 *  - limit the current to 500mA.
 *  - ...
 *
 * Even though not all features are implemented, the protocol already allows
 * for them so that its easy to migrate.
 *
 * Protocol
 * The protocol between the orb and the host on the other end of the USB
 * allows for commands described in enum Request.
 *
 * --- ORB_SET_SEQUENCE (host -> orb). Always implemented. ---
 * The protocol for setting a sequence of colors is the number of sequence
 * elements followed by 5 byte structs containing the color and times.
 * ----------
 *  uchar sequence_elements;
 *  struct {
 *      uchar red;
 *      uchar green;
 *      uchar blue;
 *      uchar morph_time;  // time to morph to the this color in 250ms steps
 *      uchar hold_time;   // time to hold this color in 250ms steps
 *  }  sequence[sequence_elements];
 *  -----------
 *  The whole sequence is repeated by the orb as soon as the data has been
 *  transmitted.
 *  If the sequence only contains one element, then hold_time and morph_time
 *  are effectivly irrelevant.
 *
 * --- ORB_GETCAPABILITIES (orb -> host). Always implemented. ---
 * Returns a struct capabilities_t that contains flags that describe the
 * capabilities of this orb and the number of elements it allows in a sequence.
 * (The simplest solution should allow for one element in the sequence which
 *  basically means: set color)
 *
 * --- ORB_SETAUX (host -> orb). Only implemented if HAS_AUX. ---
 * If capability HAS_AUX, then the single written byte corresponds to the
 * bits on the aux port.
 *
 * --- ORB_GETCOLOR (orb -> host). Only implemented if HAS_GET_COLOR. ---
 * Get the currently displayed color. If this orb is morphing between two
 * colors, then this is the current intermediate color. Returns three bytes
 * with red, green, blue.
 *
 * --- ORB_GETSEQUENCE (orb -> host). Only implemented if HAS_GET_SEQUENCE. ---
 * Returns the full sequence currently being handled in the same format as
 * used in ORB_SET_SEQUENCE.
 */

#define AVR_MHZ 12
#define F_CPU (AVR_MHZ * 1000000UL)

#include <inttypes.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "usb.h"

typedef unsigned char	uchar;
typedef unsigned short	ushort;
typedef unsigned char	bool;

#define true  1
#define false 0
#define MAX_SEQUENCE_LEN 8

// emperically determined PWM frequency: used to calculate what a morph-step is.
#define PWM_FREQUENCY_HZ 200

enum Request {
    ORB_SETSEQUENCE,
    ORB_GETCAPABILITIES,
    ORB_SETAUX,
    ORB_GETCOLOR,
    ORB_GETSEQUENCE,
};

enum CapabilityFlags {
    HAS_GET_COLOR      = 0x01,
    HAS_GET_SEQUENCE   = 0x02,
    HAS_AUX            = 0x04,
    HAS_GAMMA_CORRECT  = 0x08
    /* more to come */
};
struct capabilities_t {
    uchar flags;
    uchar max_sequence_len;
    uchar version;
    uchar reserved2;
};

struct rgb_t {
    uchar red;
    uchar green;
    uchar blue;
};

struct sequence_t {
    struct rgb_t col;
    uchar morph_time;  // time to morph to the this color in 250ms steps
    uchar hold_time;   // time to hold this color in 250ms steps
};

struct fixvalue_t {
    uint32_t value;   // upper 8 bit: value.
    int32_t diff;    // changes do be done.
};

struct morph_t {
    struct fixvalue_t red;
    struct fixvalue_t green;
    struct fixvalue_t blue;
    ushort morph_iterations;
    ushort hold_iterations;
};

// We prefill our sequence elements with the Google colors
// as the switch-on sequence
static uchar sequence_elements = 8;
static struct sequence_t sequence[ MAX_SEQUENCE_LEN ] = {
    { { 0x00, 0x00, 0x00 }, 0, 2 },   // initial black.
    { { 0x00, 0x00, 0xff }, 1, 2 },   // G - blue
    { { 0xff, 0x00, 0x00 }, 1, 2 },   // o - red
    { { 0xff, 0xff, 0x00 }, 1, 2 },   // o - yellow
    { { 0x00, 0x00, 0xff }, 1, 2 },   // g - blue
    { { 0x00, 0xff, 0x00 }, 1, 2 },   // l - green
    { { 0xff, 0x00, 0x00 }, 1, 2 },   // e - red
    { { 0x00, 0x00, 0x00 }, 1, 255 }  // black for some time.
};

bool new_data = false;

#ifndef USBTINY_SERIAL
#  error "Specify a serial number on the commandline: make USBTINY_SERIAL=ZRH0042"
#endif

#define OUT_PORT PORTA
enum {
    // 0x01, 0x02 used by usb.

    PULLUP_USB_BIT = 0x04,

    // 0x08 - N/C

    BLUE_BIT  = 0x10,
    GREEN_BIT = 0x20,
    RED_BIT   = 0x40,
    LED_MASK  = RED_BIT | GREEN_BIT | BLUE_BIT,

    DEBUG_MASK = 0x80
};

#define DEBUG_MASK 0x80
#define PULLUP_USB_BIT 0x04

// wait up to 255 milliseconds.
static void wait_millis(uchar millis) {
    uchar i;
    for(i=0; i < millis; i++){
        _delay_ms(1.0);
    }
}

struct time_mask {
    uchar  mask;        // LED mask
    ushort time;        // point in time.
};

struct time_mask segments[4];

volatile uchar active_timing = 0;

// -- USB
static enum InputMode { NO_INPUT, SET_COLOR, SET_AUX } input_mode = NO_INPUT;
extern byte_t usb_setup ( byte_t data[8] )
{
    uchar retval = 0;
    switch (data[1]) {
    case ORB_SETSEQUENCE: 
        input_mode = SET_COLOR;
        // receive data in usb_out();
        break;

    case ORB_GETCAPABILITIES: {
        struct capabilities_t *cap = (struct capabilities_t*) data;
        cap->flags = HAS_GET_COLOR | HAS_GAMMA_CORRECT;
        cap->max_sequence_len = MAX_SEQUENCE_LEN;
        cap->version = 1;
        cap->reserved2 = 0;
        retval = sizeof(*cap);
        break;
    }

    case ORB_GETCOLOR:
#if 0
        data[0] = next_rgb[read_buffer][0];
        data[1] = next_rgb[read_buffer][1];
        data[2] = next_rgb[read_buffer][2];
#endif
        retval = 3;
        break;

    case ORB_SETAUX:
        input_mode = SET_AUX;
        break;

    default:
        ; // not handled.
    }
    return retval;
}

struct fill_tracker {
    uchar sequence_elements;
    char* data;
    uchar bytes_left;
} ft;

extern void usb_out( byte_t *data, byte_t len) {
    switch (input_mode) {
    case SET_COLOR: {
        if (ft.bytes_left == 0) {  // new start.
            ft.sequence_elements = ((data[0] <= MAX_SEQUENCE_LEN)
                                              ? data[0]
                                    : MAX_SEQUENCE_LEN);
            ++data; --len;  // consumed first byte.
            ft.data = (char*) &sequence;
            ft.bytes_left = ft.sequence_elements * sizeof(struct sequence_t);
        }
        if (ft.bytes_left >= len) {
            memcpy(ft.data, data, len);
            ft.bytes_left -= len;
            ft.data += len;
        }
        if (ft.bytes_left == 0) {
            new_data = true;
            sequence_elements = ft.sequence_elements;
            input_mode = NO_INPUT;
        }
    }
        break;

    default:
        ;
    }
}

#if 0
static void timer_init() {
    // WGM1 = 15 (fast pwm, OCR1A TOP)
    TCCR1A = (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1<<WGM13) | (1<<WGM12)
        //| (1<<CS12);   // 256 prescale, page 109
        | (1<<CS11) | (1<<CS10);   // 64 prescale, page 109
        //| (1<<CS11);   // 8 prescale, page 109
    // enable timer 1 Overflow
    TIMSK1 = (1<<TOIE1);
}
#endif

static void swap(struct time_mask *a, struct time_mask *b) {
    static struct time_mask tmp;  // not on stack. Makes accounting simpler.
    tmp = *b;
    *b = *a;
    *a = tmp;
}

// for 3 elements, bubblesort is really the simplest and best.
static void sort(struct time_mask *a, int count) {
    int i, j;
    for (i = 0; i < count; ++i) {
        for (j = i + 1; j < count; ++j) {
            if (a[i].time > a[j].time) {
                swap(&a[i], &a[j]);
            }
        }
    }
}

// Set color with already gamma corrected short values [0..1023]
void set_color(ushort r, ushort g, ushort b, struct time_mask *target) {
    target[0].mask = PULLUP_USB_BIT | (r > 0 ? RED_BIT : 0);
    target[0].time = 1023 - r;

    target[1].mask = PULLUP_USB_BIT | (g > 0 ? GREEN_BIT : 0);
    target[1].time = 1023 - g;

    target[2].mask = PULLUP_USB_BIT | (b > 0 ? BLUE_BIT : 0);
    target[2].time = 1023 - b;

    // Sort in sequence when it has to be switched on (unroll?)
    sort(target, 3);

    // Each segment has the bits set from the one before.
    target[1].mask |= target[0].mask;
    target[2].mask |= target[1].mask;
}

// Gamma correct and scale input [0..255] -> [0..1023]
static ushort gamma_correct(uchar c) {
    // Simplified gamma ~= 2.2
    if (c < 64) return c;
    if (c < 128) return (c - 63) * 3 + 63;
    return (c - 127) * 6 + 255;
}

static void set_rgb(uchar r, uchar g, uchar b) {
    set_color(gamma_correct(r), gamma_correct(g), gamma_correct(b), segments);
}

void set_fixvalue_register(int32_t from, int32_t to, int32_t iterations,
                           struct fixvalue_t *out) {
    if (iterations != 0) {
        out->value = from << 21;
        out->diff = (to - from) * 2048 * 1024 / iterations;
    } else {
        out->value = to << 21;   // unsigned mul with 2048 * 1024
    }
}

static inline void increment_fixvalue(struct fixvalue_t *value) {
    value->value += value->diff;
}

void prepare_morph(const struct rgb_t *prev,
                   const struct sequence_t *target,
                   struct morph_t *morph) {
    morph->morph_iterations = PWM_FREQUENCY_HZ / 4 * target->morph_time;
    morph->hold_iterations = PWM_FREQUENCY_HZ / 4 * target->hold_time;
    set_fixvalue_register(prev->red, target->col.red,
                          morph->morph_iterations,
                          &morph->red);
    set_fixvalue_register(prev->green, target->col.green,
                          morph->morph_iterations,
                          &morph->green);
    set_fixvalue_register(prev->blue, target->col.blue,
                          morph->morph_iterations,
                          &morph->blue);
}

// Morph and return 'false' if next morph cycle needs to be calculated.
static bool do_morph(struct morph_t *morph) {
    set_rgb(morph->red.value >> 21,
            morph->green.value >> 21,
            morph->blue.value >> 21);
    // first we count down all morph iterations ..
    if (morph->morph_iterations != 0) {
        increment_fixvalue(&morph->red);
        increment_fixvalue(&morph->green);
        increment_fixvalue(&morph->blue);
        --morph->morph_iterations;
    }
    // .. then continue with the hold iterations.
    else if (morph->hold_iterations != 0) {
        --morph->hold_iterations;
    }
    // .. until we're done.
    else {
        return false;
    }
    return true;
}

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT | DEBUG_MASK;
    OUT_PORT = 0;

    usb_init();

    // Initialize segments. The last segment is always 1023
    segments[3].time = 1023;
    segments[3].mask = PULLUP_USB_BIT;
    set_rgb(0, 0, 0);   // the other segments are initialized with this.

    static struct morph_t morph;
    prepare_morph(&sequence[0].col, &sequence[0], &morph);

    ft.bytes_left = 0;
    ushort pwm = 0;
    uchar s = 0;
    ushort trigger = segments[s].time;

    uchar current_sequence = 0;

    OUT_PORT = PULLUP_USB_BIT;

    for (;;) {
        usb_poll();

        if (new_data) {
            if (sequence_elements > 0) {
                current_sequence = sequence_elements - 1;
            } else {
                set_rgb(sequence[0].col.red,
                        sequence[0].col.green,
                        sequence[0].col.blue);
            }
        }

        if (pwm++ >= trigger) {
            OUT_PORT = segments[s].mask;
            ++s;   // this could count backwards and compare against 0.
            if (s > 3) {
                OUT_PORT |= DEBUG_MASK;
                s = 0;
                pwm = 0;
                if ((new_data || !do_morph(&morph)) && sequence_elements > 0) {
                    new_data = false;
                    uchar next_sequence
                        = (current_sequence + 1) % sequence_elements;
                    // TODO: instead of current_seq, take current color here.
                    prepare_morph(&sequence[current_sequence].col,
                                  &sequence[next_sequence],
                                  &morph);
                    current_sequence = next_sequence;
                }
            }
            trigger = segments[s].time;
        }
    }
    return 0;
}
