#define F_CPU 3333333
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MIN(a,b) ((a) < (b) ? (a) : (b))
//create struct output_t
typedef struct {
    //pointer controlling pin direction, output, and lastly a bit map for a pin to go in
    register8_t *direction;
    register8_t *output;
    uint8_t bit_map;
} output_t;

output_t rgb_led[] = {
    //points to direction and output registers for PORTA
    {
        .direction = &PORTA.DIR,
        .output = &PORTA.OUT,
        .bit_map = PIN4_bm,
    },
    {
        .direction = &PORTA.DIR,
        .output = &PORTA.OUT,
        .bit_map = PIN5_bm,
    },
    {
        .direction = &PORTA.DIR,
        .output = &PORTA.OUT,
        .bit_map = PIN6_bm,
    },
};

// Set up TCA peripheral to generate periodic interrupts
// These will be used to bit bang PWM output on RGB LED pins
// Necessary because the AVR-BLE board doesn't offer access to 3 pins
// we can simultaneously use as PWM outputs from TCA0
void initTCA() {
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm |
            TCA_SINGLE_CMP1_bm | TCA_SINGLE_CMP2_bm;
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_NORMAL_gc;

    // Set up PWM frequency of 300Hz
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1_gc;
    TCA0.SINGLE.PER = 11111;

    // Start with 0% duty cycle for RGB LED pins
    TCA0.SINGLE.CMP0 = 0;
    TCA0.SINGLE.CMP1 = 0;
    TCA0.SINGLE.CMP2 = 0;

    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}

// Set up GPIO pins to control RGB LED
void initLED() {
    for (uint8_t i = 0; i < 3; i++) {
        *rgb_led[i].direction |= rgb_led[i].bit_map;
        *rgb_led[i].output &= ~(rgb_led[i].bit_map);
    }
}

ISR(TCA0_OVF_vect) {
    // Start a new PWM cycle - bring all pins high
    for (uint8_t i = 0; i < 3; i++) {
        *rgb_led[i].output |= rgb_led[i].bit_map;
    }
    TCA0.SINGLE.INTFLAGS &= TCA_SINGLE_OVF_bm;
    
}

ISR(TCA0_CMP0_vect) {
    // Duty cycle for first RGB LED pin (R)
    // Bring this pin low
    *rgb_led[0].output &= ~(rgb_led[0].bit_map);
    TCA0.SINGLE.INTFLAGS &= TCA_SINGLE_CMP0_bm;
}

ISR(TCA0_CMP1_vect) {
    // Duty cycle for second RGB LED pin (G)
    // Bring this pin low
    *rgb_led[1].output &= ~(rgb_led[1].bit_map);
    TCA0.SINGLE.INTFLAGS &= TCA_SINGLE_CMP1_bm;
}

ISR(TCA0_CMP2_vect) {
    // Duty cycle for third RGB LED pin (B)
    // Bring this pin low
    *rgb_led[2].output &= ~(rgb_led[2].bit_map);
    TCA0.SINGLE.INTFLAGS &= TCA_SINGLE_CMP2_bm;
}

// Set the red, green, and blue levels of an RGB LED
void setRgbLed(uint16_t red, uint16_t green, uint16_t blue) {
    // Rescale to [0, TCA0.SINGLE.PER]]
    // Sets new duty cycles for PWM signals on RGB LED pins
    TCA0.SINGLE.CMP0BUF = MIN(red, 1024) * 10;
    TCA0.SINGLE.CMP1BUF = MIN(green, 1024) * 10;
    TCA0.SINGLE.CMP2BUF = MIN(blue, 1024) * 10;
}

void initTWI() {
    // TODO Set up TWI Peripheral
    //set SCL AND SDA as output
//    PORTA.DIR |= PIN2_bm;
//    PORTA.DIR |= PIN3_bm;
    //enable pull up resistors
    PORTA.PIN2CTRL = PORT_PULLUPEN_bm;
    PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
    TWI0.MSTATUS = TWI_RIF_bm | TWI_WIF_bm | TWI_RXACK_bm | TWI_BUSSTATE_IDLE_gc;
    TWI0.MBAUD = 10;
    TWI0.MCTRLA = TWI_ENABLE_bm;
    
    
    
    
}

void readAccelerometerBytes(uint8_t *dest, uint8_t len) {
    // TODO read 'len' bytes of data from the accelerometer
    // into the memory location starting at 'dest'
    // No need for interrupts, you can just busy wait
    uint8_t count = 0;
    //pretty much tells accelerometer "i want to read data starting from 0x02, pretty much acts as a pointer
    TWI0.MADDR = (0x19 << 1) | 0; 
    while (!(TWI0.MSTATUS & TWI_WIF_bm));
    TWI0.MDATA = 0x02;
    while (!(TWI0.MSTATUS & TWI_WIF_bm));
    TWI0.MCTRLB = TWI_MCMD_REPSTART_gc; 
   
    //basically getting it prepared to read from i2c/accelerometer
    TWI0.MADDR = (0x19 << 1) | 1;
    //wait for any read operations to finish
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    //starts reading
    while(count < len){
        //TWI WAIT
        
        while (!(TWI0.MSTATUS & TWI_RIF_bm));
        dest[count] = TWI0.MDATA;
        
        
        count++;
        //keep reading until it had read all the bytes from the accelerometer
        if (count != len){
            TWI0.MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
        }
        }
    //finish when the master doesnt want to receive any more data from the accelerometer(when its done).
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
    }

int main() {
    int16_t accel[3];
    initTCA();
    initLED();
    initTWI();
    sei();

    while (1) {
        readAccelerometerBytes((uint8_t *) accel, 6);
        //once read from the accelerometer, send the date to the rgbLEDs.
        // Accelerometer values only have 12 meaningful bits
        // Need to throw out least significant 4 bits read in for each value
        accel[0] >>= 4;
        accel[1] >>= 4;
        accel[2] >>= 4;
        // x, y, and z
        setRgbLed(abs(accel[0]), abs(accel[1]), abs(accel[2]));

        _delay_ms(1000);
    }
}
