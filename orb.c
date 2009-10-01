/* USB orb with an attiny44
 * (c) 2008 Henner Zeller <h.zeller@acm.org>
 * This softare is GPL licensed.
 *
 * We have 4k flash and 256 bytes RAM (of which are already ~1700 bytes flash
 * and ~50 bytes RAM used by the USB implementation), but want all these
 * Features
 *  + setting a single color
 *  - switching the AUX port  (not anymore in this version of the Microorb)
 *
 * These features have been added since the 2k version of the initial orb:
 *  + set multiple colors and cycle between them according to the 'hold' time.
 *    (as many colors as memory allows)
 *  + do a smooth morph between adjacend colors in the cycle according to the
 *    'morph' time.
 *  + store the serial number in eeprom instead of the code.
 *  + do gamma correction by having a mapping from 255 color values to
 *    1024 PWM values; do 1024 PWM. Have the right relationships between
 *    colors so that 'white' is white.
 *  + limit the current to 500mA.
 *
 * These features have not been added.
 *  - after a setting has been active for more than 1min, store it in eeprom
 *    and restore it on startup. Only store _changed_ configuration because
 *    eeprom has a limited number of writes.
 *    (questionable if useful. It is possible though to poke in the sequence
 *    into eeprom, so can be an option for the commandline tool)
 *  - Do the PWM in an interrupt handler. (won't work because of USB timing
 *    restrictions).
 *
 * Protocol
 * The protocol between the orb and the host on the other end of the USB
 * allows for commands described in enum Request.
 *
 * --- ORB_SET_SEQUENCE (host -> orb). Always implemented. ---
 * The protocol for setting a sequence of colors is the number of sequence
 * elements followed by 5 byte structs containing the color and times.
 * ----------
 * struct sequence_t {
 *  uchar sequence_elements;
 *  struct {
 *      uchar red;
 *      uchar green;
 *      uchar blue;
 *      uchar morph_time;  // time to morph to the this color in 250ms steps
 *      uchar hold_time;   // time to hold this color in 250ms steps
 *  } sequence[sequence_elements];
 * }
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
 * bits on the aux port (not in this version of the Orb. The first one had
 * hardware for it, but was never used).
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

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#include "usb.h"

#define USE_TIMER 0

typedef unsigned char	uchar;
typedef unsigned short	ushort;
typedef unsigned char	bool;

#define true  1
#define false 0

// --- Hardware specific Configuration
// Number of elements in the sequence we support.
#define MAX_SEQUENCE_LEN 16

// Emperically determined PWM frequency: used to calculate how many iterations
// we need for one morph step (depends on the number of CPU cycles spent in the
// main-loop.
#define PWM_FREQUENCY_HZ 200

// The IO port we're writing to for color setting.
#define OUT_PORT PORTA

// Settable bits on the IO-port.
enum {
    // 0x01, 0x02 used by usb subsystem.

    PULLUP_USB_BIT = 0x04,   // Signal USB bus that we're ready.

    // 0x08 - N/C

    BLUE_BIT  = 0x10,
    GREEN_BIT = 0x20,
    RED_BIT   = 0x40,
    LED_MASK  = RED_BIT | GREEN_BIT | BLUE_BIT,

    AUX_PORT = 0x80          // auxiliary output for user hacking.
};

// ---- The following enums and structs are known as well to the host
//      program as they're used in the communication (might become an include?)
enum Request {
    ORB_SETSEQUENCE,
    ORB_GETCAPABILITIES,
    ORB_SETAUX,       // This orb does not provide SETAUX
    ORB_GETCOLOR,
    ORB_GETSEQUENCE,  // not implemented.
    ORB_POKE_EEPROM,
};

enum CapabilityFlags {
    HAS_GET_COLOR      = 0x01,
    HAS_GET_SEQUENCE   = 0x02,
    HAS_AUX            = 0x04,
    HAS_GAMMA_CORRECT  = 0x08,
    HAS_CURRENT_LIMIT  = 0x10
    /* more to come */
};

struct capabilities_t {
    uchar flags;
    uchar max_sequence_len;
    uchar version;
    uchar reserved2;
};

// A color; non-gamma corrected values from 0..255
struct rgb_t {
    uchar red;
    uchar green;
    uchar blue;
};

// A sequence element consists of the color, the time to morph to that color
// and the time to hold the color.
struct color_period_t {
    struct rgb_t col;
    uchar morph_time;  // time to morph to the this color in 250ms steps
    uchar hold_time;   // time to hold this color in 250ms steps
};

// A sequence as set by the user. We only have one sequence in memory for
// space reasons; it would be better to have two (one for writing from usb and
// one for reading) and flip between these. But we don't have enough memory for
// that. And the worst that could happen due to this 'race' is a brief color
// glitch.
struct sequence_t {
    uchar count;
    struct color_period_t period[ MAX_SEQUENCE_LEN ];
};

// ---- End host known data structures.

static struct sequence_t sequence;  // there is one global sequence.

// New sequence data arrived. Set by the USB callback once all data has been
// received and stored in the global sequence.
static bool new_sequence_data = false;

// A simple fixed-point arithmetic register used for color morphing.
// We have a high resolution increment for each iteration (which might be
// much less than 1 if we over a minute want to change from 0..255 but
// have 200Hz * 60s iterations for that). So we need a float representation that
// is cheap to compute in a microcontroller.
struct fixedpoint_t {
  union {
    uchar pre_dot;  // todo: Not tested yet; find out alignment in AVR here.
    uint32_t full_resolution;
  } value;
    int32_t scaled_diff;     // Change per step. Scaled to full resolution.
};

// Prepare the fixedpiont value to go from value 'from' to 'to' in the given
// number of 'iterations'.
static void fixedpoint_set_difference(struct fixedpoint_t *out,
                                      int32_t from, int32_t to,
                                      int32_t iterations);

// Step towards the prepared target value. After this has been called
// the number of iterations given in set_difference(), the value has reached
// the target value.
static void fixedpoint_increment(struct fixedpoint_t *value);



// The morphing data structure keeping track of the current morph state.
struct colormorph_t {
    struct fixedpoint_t red;
    struct fixedpoint_t green;
    struct fixedpoint_t blue;
    ushort morph_iterations;
    ushort hold_iterations;
};
static struct colormorph_t morph;   // Only one global morph state.

// operations on the global morph instance (passing a pointer of it uses a bit
// more program space)

// Prepare the global morph data structure to morph from the current color to
// the target color - given the morph- and hold-time request in target.
static void colormorph_prepare(const struct rgb_t *current,
                               const struct color_period_t *target);

// Morph one step, return 'false' when we're done with morphing, i.e. all
// morph and hold iterations are completed.
static bool colormorph_step(void);

// PWM segment data. see set_color() for explanation.
struct pwm_segment_t {
    uchar  mask;        // LED mask
    ushort time;        // point in time.
};
static struct pwm_segment_t pwm_segments[4];

// ---- EEPROM configuration ----
// (there is the serial number configuration earlier in eeprom memory as
//  linked from usb.c)

// Only if the current_limit_config contains this magic value, we actually
// switch it off. This is for advantageous users that know that their USB hub
// is fine with sourcing more than 500mA. And know what they're doing.
#define SWITCH_CURRENT_LIMIT_OFF_MAGIC 0x2a

// Current limiting configuration. Only if this value has the right magic
// content, we switch off current limiting.
uchar ee_current_limit EEMEM = ~SWITCH_CURRENT_LIMIT_OFF_MAGIC;

// Initial sequence - stored in eeprom. We set it to the Google colors to
// have some feedback when plugging in the USB.
// With the POKE_EEPROM it can be set to anything by a knowledgable user ;)
static struct sequence_t ee_initial_sequence EEMEM = {
    16,
    {
        { { 0x00, 0x00, 0x00 }, 0, 2 },   // initially briefly black.
        { { 0x00, 0x00, 0xff }, 1, 2 },   // G - blue
        { { 0xff, 0x00, 0x00 }, 1, 2 },   // o - red
        { { 0xff, 0xff, 0x00 }, 1, 2 },   // o - yellow
        { { 0x00, 0x00, 0xff }, 1, 2 },   // g - blue
        { { 0x00, 0xff, 0x00 }, 1, 2 },   // l - green
        { { 0xff, 0x00, 0x00 }, 1, 2 },   // e - red
        { { 0x00, 0x00, 0x00 }, 1, 255 }, // black for some time...
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
    }
};
// ---- end EEPROM configuration ----

// Global flag if we do current limiting. That might be switched off at
// boot time if the ee_current_limit configuration value suggests so.
static bool do_current_limit = true;

// -- USB
static enum InputMode {
    NO_INPUT,
    SET_SEQUENCE,
    SET_AUX,
    POKE_EEPROM
} input_mode = NO_INPUT;

extern byte_t usb_setup ( byte_t data[8] )
{
    uchar retval = 0;
    switch (data[1]) {
    case ORB_SETSEQUENCE:
        input_mode = SET_SEQUENCE;
        // retval=0: receive data in usb_out();
        break;

    case ORB_GETCAPABILITIES: {
        struct capabilities_t *cap = (struct capabilities_t*) data;
        cap->flags = HAS_GET_COLOR | HAS_GAMMA_CORRECT
            | (do_current_limit ? HAS_CURRENT_LIMIT : 0);
        cap->max_sequence_len = MAX_SEQUENCE_LEN;
        cap->version = 1;
        cap->reserved2 = 0;
        retval = sizeof(*cap);
        break;
    }

    case ORB_GETCOLOR:
        data[0] = morph.red.value.pre_dot;
        data[1] = morph.green.value.pre_dot;
        data[2] = morph.blue.value.pre_dot;
        retval = 3;
        break;

    case ORB_POKE_EEPROM:
        input_mode = POKE_EEPROM;
        // retval=0: receive data in usb_out();
        break;

    default:
        ; // not handled.
    }
    return retval;
}


// Buffer to keep track of read data that comes in chunks of 8 bytes
// in usb_out()
static struct read_buffer_t {
    uchar count;
    char* data;
    ushort data_left;
    ushort bytes_left;  // that can be different if the host sends too many.
} rbuf;

extern void usb_out( byte_t *data, byte_t len) {
    switch (input_mode) {
    case SET_SEQUENCE: {
        if (rbuf.bytes_left == 0) {  // new start.
            const uchar host_len = data[0];
            ++data; --len;  // consumed first length byte.
            rbuf.count = ((host_len <= MAX_SEQUENCE_LEN)
                          ? host_len
                          : MAX_SEQUENCE_LEN);
            // We override the sequence we currently have in memory. This is a
            // benign race; worst that could happen is a wrong color briefly.
            rbuf.data = (char*) &sequence.period;
            rbuf.data_left = rbuf.count * sizeof(struct color_period_t);
            // Actual number of bytes might be bigger if the host sends more
            // colors than we can handle. Be graceful and just ignore it.
            rbuf.bytes_left = host_len * sizeof(struct color_period_t);
        }
        if (rbuf.data_left > 0) {
            const uchar to_read = rbuf.data_left > len ? len : rbuf.data_left;
            memcpy(rbuf.data, data, to_read);
            rbuf.data_left -= to_read;
            rbuf.data += to_read;
        }
        if (rbuf.bytes_left >= len) {
            rbuf.bytes_left -= len;
        }
        if (rbuf.bytes_left == 0) {
            sequence.count = rbuf.count;
            input_mode = NO_INPUT;
            new_sequence_data = true;
        }
        break;
    }

    case POKE_EEPROM: {
        int pos = data[0];  // Position in eeprom.
        int i;
        for (i = 1; i < len; ++i, ++pos) {
            eeprom_write_byte((byte_t*)pos, data[i]);
        }
        break;
    }

    default:
        ;
    }
}

static void swap(struct pwm_segment_t *a, struct pwm_segment_t *b) {
  static struct pwm_segment_t tmp;  // not on stack. Makes accounting simpler.
  tmp = *b;
  *b = *a;
  *a = tmp;
}

// for 3 elements, bubblesort is really the simplest and best ;)
static void sort(struct pwm_segment_t *a, int count) {
    int i, j;
    for (i = 0; i < count; ++i) {
        for (j = i + 1; j < count; ++j) {
            if (a[i].time > a[j].time) {
                swap(&a[i], &a[j]);
            }
        }
    }
}

// Set color with already gamma corrected short values [0..1023].
//
// Setting the brightness of the LED is done by setting the on/off duty
// cycle of it (PWM).. which we loop through at around 200Hz so there is
// no flicker.
// Unfortunately, we cannot use fast hardware PWM because the Attiny only has
// two hardware PWM output pins (to which two of the outputs are connected so
// it is tinkerable if you want to play with only two colors ;) ).
//
// We have to influence the duty cycle of 3 independent LEDs. We do this by
// dividing the whole PWM cycle in 4 segments whose length are precomputed by
// this function.
//
// We start with all LEDs black. The first segment has a 'trigger time' at
// which it switches the first LED on; in the second segment the second
// brightest base-color is switched on (the first LED keeps running) and so
// on until we have all required LEDs switched on with the trigger of the
// third segment. We keep running until the last segment, that always has the
// trigger time 1023, switches all LEDs off. New cycle begins.
//
// So setting the color basically means to prepare the segments and sort them
// by duration the LEDs are on - longest first; or: smallest trigger time first.
void set_color(uint32_t r, uint32_t g, uint32_t b,
               struct pwm_segment_t *target) {
    // Current limiting for 'blue' to produce better white. To have cheaper
    // production, we use the same value for the current limiting resistors
    // everywhere .. so we need to adjust in firmware ;)
    // +1 so that 1 does not become 0.
    b = 9 * (b+1) / 10;

    // A single color takes around 320mA full on. However we're allowed to
    // draw at most 500mA from the USB bus; so if we're beyond that, we need
    // to scale down a bit to be within the limit.
    // This will create some saturation effect when we approach that limit and
    // watch some color change but better than burning your USB port. And nicer
    // that you can get more brightness if you're only using a single color.
    const uint32_t current_limit = 1024L * 500 / 320;
    const uint32_t current_sum = r + g + b;
    if (do_current_limit && current_sum > current_limit) {
        const uint32_t factor = 1024L * current_limit / current_sum;
        r = r * factor / 1024L;
        g = g * factor / 1024L;
        b = b * factor / 1024L;
    }

    target[0].mask = PULLUP_USB_BIT | (r > 0 ? RED_BIT : 0);
    target[0].time = 1023 - r;

    target[1].mask = PULLUP_USB_BIT | (g > 0 ? GREEN_BIT : 0);
    target[1].time = 1023 - g;

    target[2].mask = PULLUP_USB_BIT | (b > 0 ? BLUE_BIT : 0);
    target[2].time = 1023 - b;

    // This is always last. No need to have that in the subsequent sort.
    target[3].time = 1023;
    target[3].mask = PULLUP_USB_BIT;

    // Sort in sequence when it has to be switched on.
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
    set_color(gamma_correct(r), gamma_correct(g), gamma_correct(b),
              pwm_segments);
}

static void fixedpoint_set_difference(struct fixedpoint_t *out,
                                      int32_t from, int32_t to,
                                      int32_t iterations) {
    if (iterations != 0) {
        out->value.full_resolution = 0;
        out->value.pre_dot = from;

        // The diff-field stores the signed difference * 2^24.
        // While computing, we don't want the _signed_ range to overflow
        // by a difference of 255 << 24.
        // Since we always have more than a couple of iterations (smallest
        // time 250ms translates to around 50 iterations), we will not overflow
        // _after_ dividing by the iterations.
        //
        // So multiply the difference first by a high enough factor to maintain
        // precision, but low enough not to overflow the temporary sint32 result.
        // Then multiply to get the right range. We loose 2 bit precision, but
        // that is fine for what we need.
        // (60 seconds * 200Hz means we need to be able to represent 1/12000
        // as fraction which we easily do).
        out->scaled_diff = (to - from) * 1024 * 4096 / iterations * 4;
    } else {
        out->value.full_resolution = 0;
        out->value.pre_dot = to;
        // diff is never used in the iterations == 0 case; don't bother setting.
    }
}

static inline void fixedpoint_increment(struct fixedpoint_t *value) {
    value->value.full_resolution += value->scaled_diff;
}

static void colormorph_prepare(const struct rgb_t *current,
                               const struct color_period_t *target) {
    // PWM_FREQUENCY_HZ / 4, because times are in 250ms steps.
    morph.morph_iterations = PWM_FREQUENCY_HZ / 4 * target->morph_time;
    morph.hold_iterations  = PWM_FREQUENCY_HZ / 4 * target->hold_time;
    fixedpoint_set_difference(&morph.red, current->red, target->col.red,
                              morph.morph_iterations);
    fixedpoint_set_difference(&morph.green, current->green, target->col.green,
                              morph.morph_iterations);
    fixedpoint_set_difference(&morph.blue, current->blue, target->col.blue,
                              morph.morph_iterations);
}

// Morph and return 'false' if next morph cycle needs to be calculated.
static bool colormorph_step(void) {
    set_rgb(morph.red.value.pre_dot,
            morph.green.value.pre_dot,
            morph.blue.value.pre_dot);
    // first we count down all morph iterations ..
    if (morph.morph_iterations != 0) {
        fixedpoint_increment(&morph.red);
        fixedpoint_increment(&morph.green);
        fixedpoint_increment(&morph.blue);
        --morph.morph_iterations;
    }
    // .. then continue with the hold iterations.
    else if (morph.hold_iterations != 0) {
        --morph.hold_iterations;
    }
    // .. until we're done.
    else {
        return false;
    }
    return true;
}

#if USE_TIMER
static void timer_init() {
    TCCR1B = (1<<CS11) | (1<<CS10);   // 64 prescale, page 109; around 183Hz
}
#endif

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT | AUX_PORT;
    OUT_PORT = 0;

    // Copy initial sequence from eeprom to memory.
    byte_t *src = (byte_t*) &ee_initial_sequence;
    byte_t *dst = (byte_t*) &sequence;
    byte_t i;
    for (i = 0; i < sizeof(struct sequence_t); ++i, ++src, ++dst)
        *dst = eeprom_read_byte(src);

    // See if we have the magic value that switches off USB-bus saving current
    // limiting...
    if (eeprom_read_byte(&ee_current_limit) == SWITCH_CURRENT_LIMIT_OFF_MAGIC)
        do_current_limit = false;

    usb_init();
#if USE_TIMER
    timer_init();
#endif

    set_rgb(0, 0, 0);
    colormorph_prepare(&sequence.period[0].col, &sequence.period[0]);

    rbuf.bytes_left = 0;
    rbuf.data_left = 0;
    uchar s = 0;
    ushort trigger = pwm_segments[s].time;

    uchar current_sequence = 0;

#if USE_TIMER
#   define PWM_RESET  TCNT1 = 0
#   define PWM_ACCESS TCNT1
#else
    ushort pwm;
#   define PWM_RESET  pwm = 0
#   define PWM_ACCESS pwm++
#endif

    PWM_RESET;

    // After the PULLUP, the usb negotiation begins. So do this after the
    // expensive setup and right before our loop.
    OUT_PORT = PULLUP_USB_BIT;

    for (;;) {
        usb_poll();

        if (new_sequence_data) {
            if (sequence.count > 0) {
                current_sequence = sequence.count - 1;
            } else {
                set_rgb(sequence.period[0].col.red,
                        sequence.period[0].col.green,
                        sequence.period[0].col.blue);
            }
        }

        if (PWM_ACCESS >= trigger) {
            OUT_PORT = pwm_segments[s].mask;
            ++s;   // this could count backwards and compare against 0.
            if (s > 3) {
                OUT_PORT |= AUX_PORT;
                s = 0;
                PWM_RESET;
                if ((new_sequence_data || !colormorph_step())
                    && sequence.count > 0) {
                    new_sequence_data = false;
                    uchar next_sequence
                        = (current_sequence + 1) % sequence.count;
                    // TODO: instead of current_seq, take current color here.
                    colormorph_prepare(&sequence.period[current_sequence].col,
                                       &sequence.period[next_sequence]);
                    current_sequence = next_sequence;
                }
            }
            trigger = pwm_segments[s].time;
        }
    }
    return 0;
}
