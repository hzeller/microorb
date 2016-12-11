/* -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-
 * Copyright (c) 2008 Henner Zeller <h.zeller@acm.org>
 * This software is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2, as published
 * by the Free Software Foundation <http://www.gnu.org/copyleft/>.
 *
 * USB orb with an attiny44
 * We have 4k flash and 256 bytes RAM (of which are already ~1700 bytes flash
 * and ~50 bytes RAM used by the USB implementation), but want all the following
 * Features.
 *
 * These had been in the 2k version of the Orb
 *  + setting a single color.
 *  + switching the AUX port.
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
 * For details on the protocol, see microorb-protocol.h
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
#include "../microorb-protocol.h"

#define COMPILE_ASSERT(x) uchar compile_assert_[(x) ? 0 : -1]

// if 1, use the build-in counter as PWM refernce, otherwise some counter.
#define USE_TIMER 1

// Enabling debugging will send a pulse on the AUX bit, but will make the
// aux port not very usable ;)
#define DEBUGGING_ENABLED 0

typedef unsigned char	uchar;
typedef unsigned short	ushort;
typedef unsigned char	bool;

#define true  1
#define false 0

// --- Hardware specific Configuration
// Number of elements in the sequence we support.
#define MAX_SEQUENCE_LEN 16

// PWM frequency: used to calculate how many iterations we need for one morph
// step (depends on the number of CPU cycles spent in the main-loop.
#define PWM_FREQUENCY_HZ 183

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

    AUX_BIT   = 0x80          // auxiliary output for user hacking.
};

#if DEBUGGING_ENABLED
/* aux port is as well our debugging port */
#  define DEBUG_BIT AUX_BIT
#else
#  define DEBUG_BIT 0
#endif

// A sequence as set by the user.
// We only have one sequence in memory for
// space reasons; it would be better to have two (one for writing from usb and
// one for reading) and flip between these. But we don't have enough memory for
// that. And the worst that could happen due to this 'race' is a brief color
// glitch.
static struct orb_sequence_t sequence;  // There is one global sequence.

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
        struct {
            uchar padding_[3];
            uchar pre_dot;
        } v;
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
static void colormorph_prepare(const struct orb_rgb_t *current,
                               const struct orb_color_period_t *target);

// Morph one step, return 'false' when we're done with morphing, i.e. all
// morph and hold iterations are completed.
static bool colormorph_step(void);

// PWM segment data. see set_color() for explanation.
struct pwm_segment_t {
    uchar  mask;        // LED mask
    ushort time;        // point in time.
};
static struct pwm_segment_t pwm_segments[4];

static void set_aux(bool value);
static void current_limit_init();

// ---- EEPROM configuration ----
// (there is the serial number configuration earlier in eeprom memory as
//  linked from usb.c)

// Only if the current_limit_config contains this magic value, we actually
// switch it off. This is for advantageous users that know that their USB hub
// is fine with sourcing more than 500mA. And know what they're doing.
#define SWITCH_CURRENT_LIMIT_OFF_MAGIC 0x5a

// Current limiting configuration. Only if this value has the right magic
// content, we switch off current limiting.
uchar ee_current_limit EEMEM = ~SWITCH_CURRENT_LIMIT_OFF_MAGIC;

// Initial sequence - stored in eeprom. We set it to some color sequence
// have some feedback when plugging in the USB.
// With the POKE_EEPROM it can be set to anything by a knowledgable user ;)
static struct orb_sequence_t ee_initial_sequence EEMEM = {
    12,
    {
        { { 0x00, 0x00, 0x00 }, 0, 2 },   // initially briefly black.
        { { 0xff, 0x00, 0xff }, 1, 2 },   // red
        { { 0xff, 0x55, 0x00 }, 1, 2 },   // orange
        { { 0xff, 0xff, 0x00 }, 1, 2 },   // yellow
        { { 0x00, 0x00, 0xff }, 1, 2 },   // green
        { { 0x00, 0xff, 0xff }, 1, 2 },   // green-blue
        { { 0x00, 0x00, 0xff }, 1, 2 },   // blue
        { { 0x00, 0x00, 0x00 }, 1, 255 }, // black for some time...
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },
        { { 0x00, 0x00, 0x00 }, 0, 255 },

        { { 0x00, 0x00, 0x00 }, 0, 255 },  // Not used. Just sane defaults.
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
    GET_SEQUENCE,
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

    case ORB_GETSEQUENCE:
        input_mode = GET_SEQUENCE;
        retval = 0xff;
        break;

    case ORB_GETCAPABILITIES: {
        struct orb_capabilities_t *cap = (struct orb_capabilities_t*) data;
        cap->flags = (HAS_GET_COLOR | HAS_GAMMA_CORRECT
                      | HAS_AUX | HAS_GET_SEQUENCE
                      | (do_current_limit ? HAS_CURRENT_LIMIT : 0));
        cap->max_sequence_len = MAX_SEQUENCE_LEN;
        cap->version = 1;
        cap->reserved = 0;
        retval = sizeof(*cap);
        break;
    }

    case ORB_GETCOLOR:
        data[0] = morph.red.value.v.pre_dot;
        data[1] = morph.green.value.v.pre_dot;
        data[2] = morph.blue.value.v.pre_dot;
        retval = 3;
        break;

    case ORB_SETAUX:
        input_mode = SET_AUX;
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


// Buffer to keep track of data that is transmitted in chunks of 8 bytes
// in usb_out()/usb_in()
static struct transmit_buffer_t {
    char* data;
    uchar data_left;
} txbuf;
uchar tx_sequence_count;

// Data_left must be able to hold the size of a sequence.
COMPILE_ASSERT(sizeof(struct orb_sequence_t) <= 255);

// Handle incoming data from USB.
extern void usb_out(byte_t *data, byte_t len) {
    switch (input_mode) {
    case SET_SEQUENCE: {
        if (txbuf.data_left == 0) {  // new start.
            const uchar host_len = data[0];
            ++data; --len;  // consumed first length byte.
            tx_sequence_count = ((host_len <= MAX_SEQUENCE_LEN)
                                 ? host_len
                                 : MAX_SEQUENCE_LEN);
            // We override the sequence we currently have in memory. This is a
            // benign race; worst that could happen is a wrong color briefly.
            txbuf.data = (char*) &sequence.period;
            txbuf.data_left = (tx_sequence_count
                               * sizeof(struct orb_color_period_t));
        }
        if (txbuf.data_left > 0) {
            const uchar to_read = txbuf.data_left > len ? len : txbuf.data_left;
            memcpy(txbuf.data, data, to_read);
            txbuf.data_left -= to_read;
            txbuf.data += to_read;
        }
        if (txbuf.data_left == 0) {
            sequence.count = tx_sequence_count;
            input_mode = NO_INPUT;
            new_sequence_data = true;
        }
        break;
    }

    case POKE_EEPROM: {
        int pos = data[0];  // Position in eeprom.
        byte_t i;
        for (i = 1; i < len; ++i, ++pos) {
            eeprom_write_byte((byte_t*)pos, data[i]);
        }
        current_limit_init();  // current limit changed ?
        break;
    }

    case SET_AUX:
        set_aux(data[0] & 0x01);
        break;

    default:
        ;
    }
}

// Return data back to host.
extern byte_t usb_in(byte_t *data, byte_t len) {
    byte_t retval = 0;
    switch (input_mode) {
    case GET_SEQUENCE: {
        if (txbuf.data_left == 0) {  // new start.
            txbuf.data = (char*) &sequence;
            txbuf.data_left = (sequence.count
                               * sizeof(struct orb_color_period_t) + 1);
        }
        if (txbuf.data_left > 0) {
            const uchar to_read = txbuf.data_left > len ? len : txbuf.data_left;
            memcpy(data, txbuf.data, to_read);
            txbuf.data_left -= to_read;
            if (txbuf.data_left <= 0)
                input_mode = NO_INPUT;
            txbuf.data += to_read;
            retval = to_read;
        }
        break;
    }
    default:
        ;
    }
    return retval;
}

static void swap(struct pwm_segment_t *a, struct pwm_segment_t *b) {
    struct pwm_segment_t tmp;
    tmp = *b;
    *b = *a;
    *a = tmp;
}

// for 3 elements, bubblesort is really the simplest and best ;)
static void sort(struct pwm_segment_t *a, byte_t count) {
    byte_t i, j;
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
void set_color(ushort r, ushort g, ushort b,
               struct pwm_segment_t *target) {
    // Current limiting for 'blue' to produce better white. To have cheaper
    // production, we use the same value for the current limiting resistors
    // everywhere .. so we need to adjust in firmware ;)
    // +1 so that 1 does not become 0.
    b = 14 * (b+1) / 16;
    usb_poll();

    // A single color takes around 320mA full on. However we're allowed to
    // draw at most 500mA from the USB bus; so if we're beyond that, we need
    // to scale down a bit to be within the limit.
    // This will create some saturation effect when we approach that limit and
    // watch some color change but better than burning your USB port. And nicer
    // that you can get more brightness if you're only using a single color.
    const uint32_t current_limit = 1024L * 500 / 320;
    const uint32_t current_sum = r + g + b;
    if (do_current_limit && current_sum > current_limit) {
        const ushort factor = 64L * current_limit / current_sum;
        r = r * factor / 64;
        usb_poll();
        g = g * factor / 64;
        usb_poll();
        b = b * factor / 64;
        usb_poll();
    }

    target[0].mask = (r != 0 ? RED_BIT : 0);
    target[0].time = 1023 - r;

    target[1].mask = (g != 0 ? GREEN_BIT : 0);
    target[1].time = 1023 - g;

    target[2].mask = (b != 0 ? BLUE_BIT : 0);
    target[2].time = 1023 - b;

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
    usb_poll();
    set_color(gamma_correct(r), gamma_correct(g), gamma_correct(b),
              pwm_segments);
    usb_poll();
}

static void fixedpoint_set_difference(struct fixedpoint_t *out,
                                      int32_t from, int32_t to,
                                      int32_t iterations) {
    if (iterations != 0) {
        out->value.full_resolution = 0;
        out->value.v.pre_dot = from;

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
        out->value.v.pre_dot = to;
        // diff is never used in the iterations == 0 case; don't bother setting.
    }
}

static void fixedpoint_increment(struct fixedpoint_t *value) {
    value->value.full_resolution += value->scaled_diff;
}

static void colormorph_prepare(const struct orb_rgb_t *current,
                               const struct orb_color_period_t *target) {
    // PWM_FREQUENCY_HZ / 4, because times are in 250ms steps.
    morph.morph_iterations = PWM_FREQUENCY_HZ / 4 * target->morph_time;
    morph.hold_iterations  = PWM_FREQUENCY_HZ / 4 * target->hold_time;
    fixedpoint_set_difference(&morph.red, current->red, target->color.red,
                              morph.morph_iterations);
    fixedpoint_set_difference(&morph.green, current->green, target->color.green,
                              morph.morph_iterations);
    fixedpoint_set_difference(&morph.blue, current->blue, target->color.blue,
                              morph.morph_iterations);
}

// Morph and return 'false' if next morph cycle needs to be calculated.
static bool colormorph_step(void) {
    set_rgb(morph.red.value.v.pre_dot,
            morph.green.value.v.pre_dot,
            morph.blue.value.v.pre_dot);
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

static void set_aux(bool value) {
    if (value)
        OUT_PORT |= AUX_BIT;
    else
        OUT_PORT &= ~AUX_BIT;
}

static void current_limit_init() {
    // See if we have the magic value that switches off USB-bus saving current
    // limiting...
    do_current_limit =
        (eeprom_read_byte(&ee_current_limit) != SWITCH_CURRENT_LIMIT_OFF_MAGIC);
}

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT | AUX_BIT;
    OUT_PORT = 0;

    // Copy initial sequence from eeprom to memory.
    byte_t *src = (byte_t*) &ee_initial_sequence;
    byte_t *dst = (byte_t*) &sequence;
    byte_t i;
    for (i = 0; i < sizeof(struct orb_sequence_t); ++i, ++src, ++dst)
        *dst = eeprom_read_byte(src);

    current_limit_init();
    usb_init();
#if USE_TIMER
    timer_init();
#endif

    // This is always last in the pwm_segments and is never modified. Only set
    // once.
    pwm_segments[3].time = 1023;
    pwm_segments[3].mask = DEBUG_BIT;
    set_rgb(0, 0, 0);         // initialize the rest of the fields.

    colormorph_prepare(&sequence.period[0].color, &sequence.period[0]);

    txbuf.data_left = 0;
    uchar s = 0;
    ushort next_pwm_action = pwm_segments[s].time;

    uchar current_sequence = 0;

#if USE_TIMER
#   define PWM_COUNTER_RESET  TCNT1 = 0
#   define PWM_ACCESS TCNT1
#else
    ushort pwm;
#   define PWM_COUNTER_RESET  pwm = 0
#   define PWM_ACCESS pwm++
#endif

    struct orb_rgb_t last_color;

    PWM_COUNTER_RESET;

    // After the PULLUP, the usb negotiation begins. So do this after the
    // expensive setup and right before our loop.
    OUT_PORT = PULLUP_USB_BIT;

    for (;;) {
        usb_poll();

        /*
         * This whole stuff seems to take too long time. If we access the
         * orb from the host in a loop, we sometimes get a protcol error. So
         * Figure out where we need the most time.
         * Having some usb_poll()s strayed in certainly helps ;)
         */
        if (PWM_ACCESS >= next_pwm_action) {
            OUT_PORT = pwm_segments[s].mask | (OUT_PORT & ~(LED_MASK|DEBUG_BIT));
            ++s;   // this could count backwards and compare against 0.
            if (s > 3) {

                /* If we got new sequence data in the meantime, we need to
                 * handle that.
                 *
                 * If we get a 'sequence' with only one element (which is the
                 * usual case), we just set the color and disable the whole
                 * colormorping branch. That improves the timing so that we
                 * won't have USB timing problems here.
                 */
                bool need_morph_init = false;
                if (new_sequence_data) {
                    if (sequence.count > 1) {
                        current_sequence = sequence.count - 1;
                        // Next sequence starts from current color
                        last_color.red = morph.red.value.v.pre_dot;
                        last_color.green = morph.green.value.v.pre_dot;
                        last_color.blue = morph.blue.value.v.pre_dot;
                        need_morph_init = true;
                    } else {
                        // The get-color reads from the morph structure. So
                        // initialize it here.
                        colormorph_prepare(&sequence.period[0].color,
                                           &sequence.period[0]);
                        set_rgb(sequence.period[0].color.red,
                                sequence.period[0].color.green,
                                sequence.period[0].color.blue);
                    }
                    new_sequence_data = false;
                }
                if (sequence.count > 1 &&
                    (need_morph_init || !colormorph_step())) {
                    current_sequence = (current_sequence + 1) % sequence.count;
                    colormorph_prepare(&last_color,
                                       &sequence.period[current_sequence]);
                    last_color = sequence.period[current_sequence].color;
                }

                s = 0;
                PWM_COUNTER_RESET;
            }
            next_pwm_action = pwm_segments[s].time;
        }
    }
    return 0;
}
