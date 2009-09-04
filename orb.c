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

struct hires_rgb_t {
    ushort red;
    ushort green;
    ushort blue;
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
uchar pullup_bit = 0;

#ifndef USBTINY_SERIAL
#  error "Specify a serial number on the commandline: make USBTINY_SERIAL=ZRH0042"
#endif

#define PORT_LED PORTA
enum ColorBits {
    BLUE_BIT  = 0x10,
    GREEN_BIT = 0x20,
    RED_BIT   = 0x40,

    LED_MASK = RED_BIT | GREEN_BIT | BLUE_BIT
};

#define DEBUG_MASK 0x80
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
    uchar  mask;        // LED mask
    ushort time;        // time for the _next_ segment (double buffered OCR1A)
    uchar next;
};

struct time_mask segments[4];

volatile uchar active_timing = 0;

// The USB interrupts are more important so we'll have our PWM timers be
// interruptable.
#if 0
void TIM1_OVF_vect() __attribute__((interrupt));
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

volatile struct time_mask *current;
ISR(TIM1_OVF_vect) {
    PORT_LED = current->mask;
    current = &segments[current->next];
    // The timings are double buffered, so we have to set the timer to the _next_
    // duration here.
    OCR1A = current->time;
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
    // WGM1 = 15 (fast pwm, OCR1A TOP)
    TCCR1A = (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1<<WGM13) | (1<<WGM12)
        //| (1<<CS12);   // 256 prescale, page 109
        | (1<<CS11) | (1<<CS10);   // 64 prescale, page 109
        //| (1<<CS11);   // 8 prescale, page 109
    // enable timer 1 Overflow
    TIMSK1 = (1<<TOIE1);
}

static void swap(struct time_mask *a, struct time_mask *b) {
    static struct time_mask tmp;  // not on stack. Makes accounting simpler.
    tmp = *b;
    *b = *a;
    *a = tmp;
}

static void swap_int(ushort *a, ushort *b) {
    ushort tmp;
    tmp = *b;
    *b = *a;
    *a = tmp;
}
// for 3 elements, bubblesort is really the simplest and best.
static void sort(struct time_mask *a, int count) {
    int i, j;
    for (i = 0; i < count; ++i)
        for (j = i; j < count; ++j)
            if (a[i].time < a[j].time)
                swap(&a[i], &a[j]);
}

void set_color(struct hires_rgb_t *color, struct time_mask *target) {
    // we calculate the color in place. For that, we use the 'next_time' as
    // color duration.
    target[0].mask = pullup_bit;  // 'black' - whole duration.
    target[0].time = 1023;

    target[1].mask = pullup_bit | RED_BIT;
    target[1].time = color->red;

    target[2].mask = pullup_bit | GREEN_BIT;
    target[2].time = color->green;

    target[3].mask = pullup_bit | BLUE_BIT;
    target[3].time = color->blue;

    // The colors need to be sorted: longest first. We already know, that
    // black is maximum, so we can start sorting from 1
    sort(&target[1], 3);

    // Chain the results.
    target[0].next = 1;    // check if this is shorter than loop.
    target[1].next = 2;
    target[2].next = 3;
    target[3].next = 0;

    // Each segment has the bits set from the one before.
    int i;
    for (i = 1; i < 4; ++i) {
        target[i].mask |= target[i-1].mask;
    }

    // Combine adjacent elements if possible and skip them.
    uchar s = 0;
    do {
        int i;
        for (i = s + 1; i < 4; ++i) {
            if (target[i].time == target[s].time) {
                target[s].next = target[i].next;
                target[s].mask |= target[i].mask;
            }
        }
        s = target[s].next;
    } while (s != 0);

    // Calculate the relative times and fill them in the right spots.
    s = 0;
    for (;;) {
        uchar next = target[s].next;
        if (next == 0) break;
        target[s].time = target[s].time - target[next].time;
        s = next;
    }

    // When we start, lets set the debug mask to have a sync pulse.
    target[0].mask |= DEBUG_MASK;
}

static void set_rgb(uchar r, uchar g, uchar b) {
    struct hires_rgb_t color;
    color.red = r * 4;
    color.green = g * 4;
    color.blue = b * 4;
    set_color(&color, &segments[0]);
}

static void blinkenlights() {
    set_rgb(0, 0, 255);
    wait_millis(200);
    set_rgb(255, 0, 0);
    wait_millis(200);
    set_rgb(255, 255, 0);
    wait_millis(200);
    set_rgb(0, 0, 255);
    wait_millis(200);
    set_rgb(0, 255, 0);
    wait_millis(200);
    set_rgb(255, 0, 0);
    wait_millis(200);
}

int main(void)
{

    DDRA = LED_MASK | PULLUP_USB_BIT | DEBUG_MASK;

    PORT_LED &= ~PULLUP_USB_BIT;

    struct hires_rgb_t color;

    usb_init();

    current = &segments[0];

    //timer_init();
    //OCR1A = 1;

    //blinkenlights();

    // Now, enable the pullup bit for USB in all following color settings.
    pullup_bit = PULLUP_USB_BIT;
    set_rgb(0, 0, 0);

    current[0].time = 7;
    current[0].mask = PULLUP_USB_BIT | RED_BIT;
    current[0].next = 1;

    current[1].time = 512;
    current[1].mask = PULLUP_USB_BIT;
    current[1].next = 0;

    int pwm = 0;
    int trigger = current->time;
    for (;;) {
        usb_poll();
        if (new_data) {
            color.red = sequence[0].red * 4;
            color.green = sequence[0].green * 4;
            color.blue = sequence[0].blue * 4;
            set_color(&color, &segments[0]);
            new_data = false;
        }

        if (pwm >= trigger) {
            PORT_LED = current->mask;
            trigger = current->time;
            pwm = 0;
            current = &segments[current->next];
        }
        pwm++;
    }
    return 0;
}
