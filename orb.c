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

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#include "usb.h"

#define MAX_SEQUENCE_LEN 16

typedef unsigned char	uchar;
typedef unsigned short	ushort;
typedef unsigned char	bool;

#define true  1
#define false 0

// emperically determined PWM frequency: used to calculate what a morph-step is.
#define PWM_FREQUENCY_HZ 200

// Only if the current_limit_config contains the magic value, we actually
// switch it off.
#define SWITCH_OFF_MAGIC 0x2a
uchar eeprom_current_limit_config EEMEM = 0xa5;

bool do_current_limit = true;

enum Request {
    ORB_SETSEQUENCE,
    ORB_GETCAPABILITIES,
    ORB_SETAUX,
    ORB_GETCOLOR,
    ORB_GETSEQUENCE,  // not implemented.
    ORB_POKE_EEPROM,
};

enum CapabilityFlags {
    HAS_GET_COLOR      = 0x01,
    HAS_GET_SEQUENCE   = 0x02,
    HAS_AUX            = 0x04,
    HAS_GAMMA_CORRECT  = 0x08,
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
static struct morph_t morph;

// A sequence element consists of the color, the time to morph to that color
// and the time to hold the color.
struct color_period_t {
    struct rgb_t col;
    uchar morph_time;  // time to morph to the this color in 250ms steps
    uchar hold_time;   // time to hold this color in 250ms steps
};

struct sequence_t {
    uchar count;
    struct color_period_t period[ MAX_SEQUENCE_LEN ];
};

static struct sequence_t sequence;

// Initial sequence - stored in eeprom. We set it to the Google colors.
static struct sequence_t initial_sequence EEMEM = {
    12,
    {
        { { 0x00, 0x00, 0x00 }, 0, 2 },   // initial black.
        { { 0x00, 0x00, 0xff }, 1, 2 },   // G - blue
        { { 0xff, 0x00, 0x00 }, 1, 2 },   // o - red
        { { 0xff, 0xff, 0x00 }, 1, 2 },   // o - yellow
        { { 0x00, 0x00, 0xff }, 1, 2 },   // g - blue
        { { 0x00, 0xff, 0x00 }, 1, 2 },   // l - green
        { { 0xff, 0x00, 0x00 }, 1, 2 },   // e - red
        { { 0x00, 0x00, 0x00 }, 1, 255 }, // black for some time...
        { { 0x00, 0x00, 0x00 }, 1, 255 },
        { { 0x00, 0x00, 0x00 }, 1, 255 },
        { { 0x00, 0x00, 0x00 }, 1, 255 },
        { { 0x00, 0x00, 0x00 }, 1, 255 },
    }
};

bool new_data = false;

#define OUT_PORT PORTA
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

struct time_mask {
    uchar  mask;        // LED mask
    ushort time;        // point in time.
};

struct time_mask segments[4];

volatile uchar active_timing = 0;

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
        data[0] = morph.red.value >> 21;
        data[1] = morph.green.value >> 21;
        data[2] = morph.blue.value >> 21;
        retval = 3;
        break;

    case ORB_POKE_EEPROM:
        input_mode = POKE_EEPROM;
        break;

    default:
        ; // not handled.
    }
    return retval;
}

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
            new_data = true;
        }
    }
        break;

    case POKE_EEPROM: {
        int pos = data[0];  // Position in eeprom.
        int i;
        for (i = 1; i < len; ++i, ++pos)
            eeprom_write_byte((byte_t*)pos, data[i]);
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
    //static struct time_mask tmp;  // not on stack. Makes accounting simpler.
    //struct time_mask *tmp = &segments[3];
    segments[3] = *b;
    *b = *a;
    *a = segments[3];
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
void set_color(uint32_t r, uint32_t g, uint32_t b, struct time_mask *target) {
    // Current limiting for 'blue' to produce better white. To have cheaper
    // production, we use the same value for the current limiting resistors
    // everywhere .. so we need to adjust in firmware ;)
    // +1 so that 1 does not become 0.
    b = 9 * (b+1) / 10;

    // A single color takes around 350mA full on. However we're allowed to
    // draw at most 500mA from the USB bus; so if we're beyond that, we need
    // to scale down a bit to be within the limit.
    // This will create some saturation effect when we approach that limit and
    // watch some color change.
    //
    // Some measurements show that if more colors are on, the current per LED
    // is more like 320mA, so lets take that as baseline.
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

    // Sort in sequence when it has to be switched on. So smallest values
    // first.
    sort(target, 3);

    // This is always last. We need to set it here because target[3] was
    // used as temporary swap buffer.
    target[3].time = 1023;
    target[3].mask = PULLUP_USB_BIT;

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
                   const struct color_period_t *target) {
    morph.morph_iterations = PWM_FREQUENCY_HZ / 4 * target->morph_time;
    morph.hold_iterations = PWM_FREQUENCY_HZ / 4 * target->hold_time;
    set_fixvalue_register(prev->red, target->col.red,
                          morph.morph_iterations,
                          &morph.red);
    set_fixvalue_register(prev->green, target->col.green,
                          morph.morph_iterations,
                          &morph.green);
    set_fixvalue_register(prev->blue, target->col.blue,
                          morph.morph_iterations,
                          &morph.blue);
}

// Morph and return 'false' if next morph cycle needs to be calculated.
static bool do_morph(void) {
    set_rgb(morph.red.value >> 21,
            morph.green.value >> 21,
            morph.blue.value >> 21);
    // first we count down all morph iterations ..
    if (morph.morph_iterations != 0) {
        increment_fixvalue(&morph.red);
        increment_fixvalue(&morph.green);
        increment_fixvalue(&morph.blue);
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

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT | AUX_PORT;
    OUT_PORT = 0;

    // Get initial sequence from eeprom.
    byte_t *src = (byte_t*) &initial_sequence;
    byte_t *dst = (byte_t*) &sequence;
    byte_t i;
    for (i = 0; i < sizeof(struct sequence_t); ++i, ++src, ++dst)
        *dst = eeprom_read_byte(src);

    // See if we have the magic value that switches off USB-bus saving current
    // limiting...
    if (eeprom_read_byte(&eeprom_current_limit_config) == SWITCH_OFF_MAGIC)
        do_current_limit = false;

    usb_init();

    set_rgb(0, 0, 0);   // the other segments are initialized with this.

    prepare_morph(&sequence.period[0].col, &sequence.period[0]);

    rbuf.bytes_left = 0;
    rbuf.data_left = 0;
    ushort pwm = 0;
    uchar s = 0;
    ushort trigger = segments[s].time;

    uchar current_sequence = 0;

    OUT_PORT = PULLUP_USB_BIT;

    for (;;) {
        usb_poll();

        if (new_data) {
            if (sequence.count > 0) {
                current_sequence = sequence.count - 1;
            } else {
                set_rgb(sequence.period[0].col.red,
                        sequence.period[0].col.green,
                        sequence.period[0].col.blue);
            }
        }

        if (pwm++ >= trigger) {
            OUT_PORT = segments[s].mask;
            ++s;   // this could count backwards and compare against 0.
            if (s > 3) {
                OUT_PORT |= AUX_PORT;
                s = 0;
                pwm = 0;
                if ((new_data || !do_morph()) && sequence.count > 0) {
                    new_data = false;
                    uchar next_sequence
                        = (current_sequence + 1) % sequence.count;
                    // TODO: instead of current_seq, take current color here.
                    prepare_morph(&sequence.period[current_sequence].col,
                                  &sequence.period[next_sequence]);
                    current_sequence = next_sequence;
                }
            }
            trigger = segments[s].time;
        }
    }
    return 0;
}
