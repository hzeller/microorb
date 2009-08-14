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
#define MAX_SEQUENCE_LEN 5

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
    /* suggested, not yet used */
    HAS_MORPH          = 0x08,
    DOES_GAMMA_CORRECT = 0x10
    /* more to come */
};
struct capabilities_t {
    uchar flags;
    uchar max_sequence_len;
    uchar version;
    uchar reserved2;
};

uchar sequence_elements = 0;
struct sequence_t {
    uchar red;
    uchar green;
    uchar blue;
    uchar morph_time;  // time to morph to the this color in 250ms steps
    uchar hold_time;   // time to hold this color in 250ms steps
};

struct sequence_t sequence[ MAX_SEQUENCE_LEN ];

bool new_data = false;

#ifndef USBTINY_SERIAL
#  error "Specify a serial number on the commandline: make USBTINY_SERIAL=ZRH0042"
#endif

#define PORT_LED PORTA
enum ColorBits {
    BLUE_BIT  = 0x10,
    RED_BIT   = 0x08,
    GREEN_BIT = 0x20,

    LED_MASK = RED_BIT | GREEN_BIT | BLUE_BIT
};

#define DEBUG_MASK 0x40
#define NOP_MASK 0xff
#define PULLUP_USB_BIT 0x04

/*
 * On overflow of the start period (at BOTTOM), we set the LEDs to dark.
 * While this interrupt is processing, no COMPA/COMPB interrupts must be firing
 * because they might set the LEDs to ON before we've set them to OFF
 * (recursive interrupts are enabled)
 */
#define MIN_DARK_PERIOD 2

// wait up to 255 milliseconds.
static void wait_millis(uchar millis) {
    uchar i;
    for(i=0; i < millis; i++){
        _delay_ms(1.0);
    }
}

struct time_mask {
    ushort time;
    uchar  mask;
};

struct cycle_segment {
    struct time_mask a;
    struct time_mask b;
};

typedef union {
    struct time_mask raw[4];
    struct cycle_segment segment[2];
} pwm_timings_t;

volatile uchar active_timing = 0;
volatile uchar segment = 0;
pwm_timings_t timings[2];

struct cycle_segment *current_segment;

// The USB interrupts are more important so we'll have our PWM timers be
// interruptable.
#if 1
void TIM1_OVF_vect() __attribute__((interrupt));
void TIM1_COMPA_vect() __attribute__((interrupt));
void TIM1_COMPB_vect() __attribute__((interrupt));
#endif

/*
 * - Three interrupts call function event with parameter values
 * - function cli() set action sti() while (action) { }
 */

static void show_debug(bool on) {
    if (on) {
        PORT_LED = (PORT_LED & ~LED_MASK) | DEBUG_MASK;
    } else {
        PORT_LED = PORT_LED & ~DEBUG_MASK;
    }
}

void bottom_half();
enum Event {
    EVENT_OVERFLOW = 0x01,
    EVENT_TIMER_1  = 0x02,
    EVENT_TIMER_2  = 0x04
};
volatile uchar event_queue;
static void enqueue(enum Event ev) {
    cli();
    uchar before = event_queue;
    event_queue |= ev;
    sei();
    if (before) return;
    bottom_half();
}

void bottom_half() {
    while (event_queue) {
        if (event_queue & EVENT_OVERFLOW) {
            cli();
            event_queue &= ~EVENT_OVERFLOW;
            sei();
            uchar t = active_timing;
            current_segment = &timings[t].segment[segment];
            show_debug(segment == 0);
            // Now move ahead one segment and set the timers for that segment.
            // They will be double buffered and set the next time we roll over.
            segment ^= 1;
            ushort a = timings[t].segment[segment].a.time;
            ushort b = timings[t].segment[segment].b.time;
            cli();
            OCR1A = a;
            OCR1B = b;
            sei();
        }
        if (event_queue & EVENT_TIMER_1) {
            cli();
            event_queue &= ~EVENT_TIMER_1;
            sei();
            if (current_segment->a.mask != NOP_MASK)
                PORT_LED = (PORT_LED & ~LED_MASK) | current_segment->a.mask;
        }
        if (event_queue & EVENT_TIMER_2) {
            cli();
            event_queue &= ~EVENT_TIMER_2;
            sei();
            if (current_segment->b.mask != NOP_MASK)
                PORT_LED = (PORT_LED & ~LED_MASK) | current_segment->b.mask;
        }
    }
}

ISR(TIM1_OVF_vect) {
    enqueue(EVENT_OVERFLOW);
}

ISR(TIM1_COMPA_vect) {   // Counter 1 (16 bit)
    enqueue(EVENT_TIMER_1);
}

ISR(TIM1_COMPB_vect) {   // Counter 1 (16 bit)
    enqueue(EVENT_TIMER_2);
}

// -- USB
static enum InputMode { NO_INPUT, SET_COLOR, SET_AUX } input_mode = NO_INPUT;
extern byte_t usb_setup ( byte_t data[8] )
{
    uchar retval = 0;
    switch (data[1]) {
    case ORB_SETSEQUENCE: 
        input_mode = SET_COLOR;
        break;

    case ORB_GETCAPABILITIES: {
        struct capabilities_t *cap = (struct capabilities_t*) data;
        cap->flags = HAS_GET_COLOR;
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


extern void usb_out( byte_t *data, byte_t len) {
    switch (input_mode) {
    case SET_COLOR: {
        if (len == 0 || data[0] * sizeof(struct sequence_t) != len - 1)
            return;
        sequence_elements = ((data[0] <= MAX_SEQUENCE_LEN)
                             ? data[0]
                             : MAX_SEQUENCE_LEN);
        memcpy(sequence, &data[1],
               sequence_elements * sizeof(struct sequence_t));
        new_data = true;
    }
        break;

    default:
        ;
    }

    input_mode = NO_INPUT;
}

static void timer_init() {
    // WGM1 = 7 (fast pwm, 1024 TOP)
    TCCR1A = (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1<<WGM12)
        | (1<<CS11) | (1<<CS10);   // 64 prescale, page 109
    // enable timer 1 interrupt compare A & B match and Overflow
    TIMSK1 = (1<<OCIE1B) | (1<<OCIE1A) | (1<<TOIE1);
}

static void swap(struct time_mask *a, struct time_mask *b) {
    static struct time_mask tmp;  // not on stack. Makes accounting simpler.
    tmp = *b;
    *b = *a;
    *a = tmp;
}

static void sanitize(bool left, struct time_mask *a) {
#if 0
    if (a->time == 1024) {
        a->time = 512;
        a->mask = NOP_MASK;
    }
#endif
    if (left) {
        if (a->time < MIN_DARK_PERIOD)
            a->time = MIN_DARK_PERIOD;
    } else {
        if (a->time > 1024 - MIN_DARK_PERIOD)
            a->time = 1024 - MIN_DARK_PERIOD;
    }
}

// Set color, with RGB in range 0..2048
static void set_color(ushort rgb[3], pwm_timings_t *timings) {
    /*
     * Three cases:
     *  1) 000   all durations less-equal 1024
     *  2) 100   two less-equal 1024, one more
     *  3) 110   two bigger than 1024, one less-equal
     *  4) 111   all durations bigger than 1024
     *
     *  1) start the first in the first segment (a), finish at end of that
     *     segment (b).
     *     The last two starts in the second segment (a',b') and runs to its
     *     end.
     *  2) start the longest in the first segment, let it run to the end of the
     *     second. Both shorter ones start in the second segment and end
     *     with it.
     *  3) Both the big ones start in the first segment and run to the end
     *     of the second. The lower one starts in the second segment and runs
     *     to its end.
     *  4) start the first two in the first segment and let them run to the
     *     end. Let the third start with the bigger of the two and end it
     *     somewhere in the second segment (could be end).
     */
    timings->raw[0].time = rgb[0];
    timings->raw[0].mask = RED_BIT;
    timings->raw[1].time = rgb[1];
    timings->raw[1].mask = GREEN_BIT;
    timings->raw[2].time = rgb[2];
    timings->raw[2].mask = BLUE_BIT;

    timings->raw[3].time = 0;
    timings->raw[3].mask = NOP_MASK;

    // Sorting three elements - one of the rare cases in which bubblesort makes
    // sense. And uses least amount of code.
    // Biggest first.
    uchar i, j;
    for (j = 0; j < 3; ++j) {
        for (i = 0; i < 3; ++i) {
            if (timings->raw[i].time < timings->raw[i+1].time) {
                swap(&timings->raw[i], &timings->raw[i+1]);
            }
        }
    }

    // combine adjacent
#if 1
    for (i = 2; i != 255; --i) {
        if (timings->raw[i].time == timings->raw[i+1].time) {
            timings->raw[i].mask |= timings->raw[i+1].mask;
            timings->raw[i+1].mask = timings->raw[i].mask;
        }
    }
#endif

    if (timings->raw[0].time < 1024) {  // case 1). Biggest value <= 1024
        timings->raw[0].time = 1023 - timings->raw[0].time;  // 1a: start

        timings->raw[3] = timings->raw[1];
        timings->raw[1].time = 1023;  // 1b: end
        timings->raw[1].mask = 0;

        timings->raw[2].time = 1023 - timings->raw[2].time;  // 2b : start
        timings->raw[3].time = 1023 - timings->raw[3].time;  // 2a : start

        // raw2 comes after raw3 so we need to merge it with that mask.
        timings->raw[2].mask |= timings->raw[3].mask;
    } else if (timings->raw[1].time < 1024) {  // case 2).
        timings->raw[0].time = 2048 - timings->raw[0].time;  // a: start
        timings->raw[3] = timings->raw[1];

        timings->raw[1].time = 512;                          // b: end
        timings->raw[1].mask = NOP_MASK;

        timings->raw[2].time = 1024 - timings->raw[2].time;
        timings->raw[2].mask |= timings->raw[0].mask;
        timings->raw[3].time = 1024 - timings->raw[3].time;
        timings->raw[3].mask |= timings->raw[2].mask;
    } else if (timings->raw[2].time < 1024) {  // case 3).
        timings->raw[0].time = 2048 - timings->raw[0].time; // a: start
        timings->raw[1].time = 2048 - timings->raw[1].time; // b: start
        timings->raw[1].mask |= timings->raw[0].mask;

        timings->raw[2].time = 1024 - timings->raw[2].time;
        timings->raw[3] = timings->raw[2];  // nop. same as first timer.
    } else {   // case 4
        timings->raw[0].time = 2048 - timings->raw[0].time; // a: start first
        timings->raw[0].mask |= timings->raw[2].mask;       //    .. and third
        timings->raw[1].time = 2048 - timings->raw[1].time; // b: start 2nd
        timings->raw[1].mask |= timings->raw[0].mask;

        ushort we_run_already = 1024 - timings->raw[0].time;
        timings->raw[2].time = timings->raw[2].time - we_run_already;
        timings->raw[2].mask = timings->raw[1].mask & ~timings->raw[2].mask;
        timings->raw[3] = timings->raw[2];  // nop. same as other timer.
    }

#if 0
    // Manual setting.
    timings->raw[0].time = 1022;
    timings->raw[0].mask = BLUE_BIT;

    timings->raw[1].time = 1023;
    timings->raw[1].mask = 0;
#endif

#if 0
    timings->raw[2].time = 400;
    timings->raw[2].mask = BLUE_BIT;

    timings->raw[3].time = 500;
    timings->raw[3].mask = 0;
#endif

    // sanitizing.
#if 0
    sanitize(true, &timings->raw[0]);
    sanitize(true, &timings->raw[1]);
    sanitize(false, &timings->raw[2]);
    sanitize(false, &timings->raw[3]);
#endif
}

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT;

    PORT_LED &= ~PULLUP_USB_BIT;

    event_queue = 0;
    segment = 0;
    active_timing = 0;
    ushort colors[] = { 40, 0, 0 };
    set_color(colors, &timings[active_timing]);

    usb_init();
    timer_init();
    //startup_blinkenlight();

    PORT_LED |= PULLUP_USB_BIT;

    for (;;) {
        usb_poll();
        if (new_data) {
            colors[0] = sequence[0].red * 8;
            colors[1] = sequence[0].green * 8;
            colors[2] = sequence[0].blue * 8;
            uchar next_active = active_timing ^ 1;
            set_color(colors, &timings[next_active]);
            active_timing = next_active;
            // TODO: set this to true and see how things flicker
            new_data = false;
        }
    }
    return 0;
}
