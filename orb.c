/* USB orb with an attiny 2313
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

#include "usb.h"

typedef unsigned char	uchar;
typedef unsigned short	ushort;
typedef unsigned char	bool;

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
    uchar reserved1;
    uchar reserved2;
};

#define true  1
#define false 0

enum Request {
    ORB_SETSEQUENCE,
    ORB_GETCAPABILITIES,
    ORB_SETAUX,
    ORB_GETCOLOR,
    ORB_GETSEQUENCE,
};

#ifndef USBTINY_SERIAL
#  error "Specify a serial number on the commandline: make USBTINY_SERIAL=ZRH0042"
#endif

// output debug signals
#undef DEBUG

#define PORT_LED PORTA
enum ColorBits {
    BLUE_BIT  = 0x10,
    RED_BIT   = 0x08,
    GREEN_BIT = 0x20,

    LED_MASK = RED_BIT | GREEN_BIT | BLUE_BIT
};

#define PULLUP_USB_BIT 0x04
#define AUX_BIT 0x04

#ifdef DEBUG
#  define DEBUG_MASK 0x02
#else
#  define DEBUG_MASK 0x00
#endif

uchar next_rgb[2][3];
uchar read_buffer;
uchar write_buffer;
uchar in_timer_interrupt;

// wait up to 255 milliseconds.
static void wait_millis(uchar millis) {
    uchar i;
    for(i=0; i < millis; i++){
        _delay_ms(1.0);
    }
}

static void set_aux(uchar value) {
    if (value)
        PORT_LED = PORT_LED | AUX_BIT;
    else
        PORT_LED = PORT_LED & ~AUX_BIT;
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

    case ORB_GETCAPABILITIES:
        data[0] = HAS_AUX | HAS_GET_COLOR;
        data[1] = 1;
        data[2] = 0;
        data[3] = 0;
        retval = sizeof(struct capabilities_t);
        break;

    case ORB_GETCOLOR:
        data[0] = next_rgb[read_buffer][0];
        data[1] = next_rgb[read_buffer][1];
        data[2] = next_rgb[read_buffer][2];
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
        // data[0] = count data
        uchar i;
        for (i = 0; i < 3; ++i)
            next_rgb[write_buffer][i] = data[i + 1];
        read_buffer = write_buffer;
        write_buffer = (write_buffer + 1) % 2;
    }
        break;

    case SET_AUX:
        set_aux(data[0] & 0x01);
        break;
        
    default:
        ;
    }

    input_mode = NO_INPUT;
}

static void startup_blinkenlight() {
    int i;

    // Google colors .. ;-)
    static uchar startup_colors[] = { BLUE_BIT, RED_BIT, RED_BIT | GREEN_BIT,
                                      BLUE_BIT, GREEN_BIT, RED_BIT };
    for (i = 0; i < sizeof(startup_colors); ++i) {
        PORT_LED = startup_colors[i];
        wait_millis(200);
    }
}

int main(void)
{
    write_buffer = 1;
    read_buffer = 0;

    DDRA = LED_MASK | PULLUP_USB_BIT | AUX_BIT;
    //DDRD = PULLUP_USB_BIT | AUX_BIT;

    PORT_LED &= ~PULLUP_USB_BIT;

    usb_init();
    //timer_init();
    startup_blinkenlight();

    PORT_LED |= PULLUP_USB_BIT;

    uchar rb = read_buffer;
    uchar last_port = 0xff;
    uchar level;
    for (level = 0;;++level) {
        usb_poll();

        /* This is a very crude PWM in the main loop as the interrupt driven
         * version would be tricky as we need to recursively enable
         * interrupts there: the USB interrupt must not be delayed for more than
         * 28 cpu cycles.
         * This is good for the first version of the orb.
         */
        if (level == 0) {
            rb = read_buffer;
            last_port = 0xff;
        }
        uchar led_port = 0;
        if (next_rgb[rb][0] > level) led_port |= RED_BIT;
        if (next_rgb[rb][1] > level) led_port |= GREEN_BIT;
        if (next_rgb[rb][2] > level) led_port |= BLUE_BIT;
        if (last_port != led_port) {
            PORT_LED = (led_port & LED_MASK) | (PORT_LED & ~LED_MASK);
            last_port = led_port;
        }
    }
    return 0;
}
