#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>

#include "uart.h"
#include "rotary.h"

/* Υπολογισμοί για την ταχύτητα δεδομένων με τον υπολογιστή. */
#define BAUD 9600UL

#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1) // Στρογγυλοποίηση.
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))   // Πραγματικά Baud.
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)    // Σφάλμα ανά 1000 μέρη.

// Αν το σφάλμα είναι μεγαλύτερο του 1% σταματά η μεταγλώττιση του κώδικα.
#if ((BAUD_ERROR<990) || (BAUD_ERROR>1010))
    #error Error at Baud rate is bigger than 1%. Aborting!
#endif

// Pins connected to encoder push switches.

#define ENC1_SW_PORT PORTD
#define ENC1_SW_DDR DDRD
#define ENC1_SW_PIN PIND
#define ENC1_SW PD6

#define ENC2_SW_PORT PORTB
#define ENC2_SW_DDR DDRB
#define ENC2_SW_PIN PINB
#define ENC2_SW PB2

#define BUFFER_SIZE 15

#define MAXVOLTS 5000
#define MAXAMPS 5000

volatile uint16_t tick=0;  // Time keeping variable in milliseconds.

volatile uint16_t now=0;   // Time reference variable.

volatile int16_t Vvalue=0; // Variable used to set the Volts.
volatile int16_t Avalue=0; // Variable used to set the Amps.

uint8_t volt10x=1; // Variable for 1x or 10x Volts increase.
uint8_t amps10x=1; // Variable for 1x or 10x Amps increase.

volatile uint8_t state_1 = 0; // Variable for encoder 1 state.
volatile uint8_t state_2 = 0; // Variable for encoder 2 state.

int crc16(const uint8_t*, int len); // Function for calculating CRC-16 MODBUS, init 0xFFFF.

uint8_t rotary_process_1(void); // Function to detect first encoder changes.
uint8_t rotary_process_2(void); // Function to detect second encoder changes.

void rotary_init(void); // Function to initialize the rotary encodes connections and pins state.

void checkV(void); // Function to check if the Volts encoder changed.
void checkA(void); // Function to check if the Amps encoder changed.

void setvolts(uint16_t); // Function for setting the Volts.
void setamps(uint16_t);  // Function for setting the Amps.

uint8_t readva(void);    // Function for reading the Volts and Amps from the module.

void millis_init(void);  // Function to initialize the timer/counter0 running.

int main(void){

    //char buffer[4];

    uint8_t toggleV = (1 ^ 10); // This is the combined toggle value.
    uint8_t toggleA = (1 ^ 10);

    volt10x = 1; // Initialize to either x1 or x10.
    amps10x = 1;

    int16_t oldVolts; // Variables to detect changes in Vols and amps.
    int16_t oldAmps;

    uart_init(UART_BAUD_SELECT(BAUD,F_CPU)); // UART initialization.

    rotary_init(); // Initialize the rotary encoders connections.

    millis_init(); // Enable the millis tick.

    sei(); // Interrupt Enabled.

    _delay_ms(1000); // Some time to settle things down.

    while (readva() != 1) {_delay_ms(100);};

    oldVolts = Vvalue;
    oldAmps  = Avalue;


    while(1){

        if (bit_is_clear(ENC1_SW_PIN,ENC1_SW)){
            _delay_ms(10); // Debounce time.
            if (bit_is_clear(ENC1_SW_PIN,ENC1_SW)){
                volt10x ^= toggleV;   // Toggles 1x or 10x.
                _delay_ms(100);
            };
        };

        if (bit_is_clear(ENC2_SW_PIN,ENC2_SW)){
            _delay_ms(10); // Debounce time.
            if (bit_is_clear(ENC2_SW_PIN,ENC2_SW)){
                amps10x ^= toggleA;   // Toggles 1x or 10x.
                _delay_ms(100);
            };
        };

       _delay_ms(10);

        if (oldVolts != Vvalue){

            if (tick-now >= 400) {
                setvolts(Vvalue);
                _delay_ms(500);
                if (readva()==1) {oldVolts=Vvalue;};
            };
        };

       if (oldAmps != Avalue){

            if (tick-now >= 400){
                setamps(Avalue);
                _delay_ms(500);
                if (readva()==1) {oldAmps=Avalue;};
            };
        };

        _delay_ms(10);

    };

    return 0;
}

uint8_t readva(void){

    uint8_t i=0;
    uint8_t j=0;
    uint8_t status=0;
    uint16_t c;
    uint8_t received[BUFFER_SIZE];

    uint8_t va[10];

    va[0]=0x01; // Slave address 01.

    va[1]=0x03; // Read register.

    va[2]=0x00; // Register start address High-byte.
    va[3]=0x00; // Register start address LOW-byte.

    va[4]=0x00; // Total bytes High-byte.
    va[5]=0x02; // Total bytes Low-byte.

    va[6]=0xC4; // CRC-16 register High-Byte
    va[7]=0x0B; // CRC-16 register Low-byte.

    uart_flush();

    for (j=0; j<=7; j++){
        uart_putc(va[j]);
        //_delay_ms(0.1); // Some delay. Not sure if needed. Helps debugging for the moment.
    };

    _delay_ms(500); // Some delay. Not sure if needed. Helps debugging at the moment.

    c=uart_getc();

    if (!(c&UART_NO_DATA)){

        while (!(c&UART_NO_DATA)){
            received[i]=(uint8_t) c;
            c=uart_getc();
            i++;

            if(i>=BUFFER_SIZE) {
                uart_flush();
                break;
            };

        };
    };

    /*
        Splinting a 16-bit integer to 2, 8-bit integers.
        E.g. 0x6BB7 or 0110101110110111.
        Low Byte. 0110101110110111 & 0000000011111111 = 10110111 or 0xB7.
        High Byte. (0110101110110111 >> 8) & 0000000011111111 = 0000000001101011 & 0000000011111111 = 01101011 or 6B.
    */

    if ( (received[7] == ((crc16(received,7)&0xFF))) && (received[8] == ((crc16(received,7)>>8)&0xFF))) {

        Vvalue = (received[3]<<8)|received[4];
        Avalue = (received[5]<<8)|received[6];

        status=1;
    };

    return status;
}

void setvolts(uint16_t mVolts){

    uint8_t volts[10];

    volts[0]=0x01; // Slave address 01.

    volts[1]=0x06; // Write single register.

    volts[2]=0x00; // Volts address High-byte.
    volts[3]=0x00; // Volts address Low-byte.

    volts[4]=(mVolts>>8)&0xFF; // mVolts High-byte.
    volts[5]=mVolts&0xFF;      // mVolts Low-Byte

    volts[6]=crc16(volts,6)&0xFF;      // CRC-16 register Low-byte.
    volts[7]=(crc16(volts,6)>>8)&0xFF; // CRC-16 register High-Byte

    for (uint8_t i=0; i<=7; i++){
        uart_putc(volts[i]);
        //_delay_ms(0.1); // Some delay. Not sure if needed. Helps debugging for the moment.
    };

}

void setamps(uint16_t mAmps){

    uint8_t amps[10];

    amps[0]=0x01; // Slave address 01.

    amps[1]=0x06; // Write single register.

    amps[2]=0x00; // Volts address High-byte.
    amps[3]=0x01; // Volts address Low-byte.

    amps[4]=(mAmps>>8)%0xFF; // mVolts High-byte.
    amps[5]=mAmps&0xFF;      // mVolts Low-Byte

    amps[6]=(crc16(amps,6)>>8)&0xFF; // CRC-16 register High-Byte
    amps[7]=crc16(amps,6)&0xFF;      // CRC-16 register Low-byte.

    for (uint8_t i=0; i<=7; i++){
        uart_putc(amps[i]);
        //_delay_ms(0.1); // Some delay. Not sure if needed. Helps debugging for the moment.
    };

}

void rotary_init() {

    ENC1_DDR_PORT &= ~((1<<ENC1_A)|(1<<ENC1_B));
    ENC1_PORT |= (1<<ENC1_A)|(1<<ENC1_B);

    ENC2_DDR_PORT &= ~((1<<ENC2_A)|(1<<ENC2_B));
    ENC2_PORT |= (1<<ENC2_A)|(1<<ENC2_B);

    ENC1_SW_DDR &= ~(1<<ENC1_SW); // Encoder 1 switch as input.
    ENC1_SW_PORT |= (1<<ENC1_SW); // Encoder 1 pin pull-up enabled.

    ENC2_SW_DDR &= ~(1<<ENC2_SW); // Encoder 2 switch as input.
    ENC2_SW_PORT |= (1<<ENC2_SW); // Encoder 2 pin pull-up enabled.

}


uint8_t rotary_process_1(void) {

    uint8_t pinstate=0;

    // pinstate 0~3.
    pinstate = (((bit_is_set(ENC1_PIN_PORT,ENC1_A)>>ENC1_A)&1)<<1)|((bit_is_set(ENC1_PIN_PORT,ENC1_B)>>ENC1_B)&1);

    state_1 = ttable[state_1 & 0xF][pinstate];

    return (state_1 & 0x30);
}

uint8_t rotary_process_2(void) {

    uint8_t pinstate=0;

    // pinstate 0~3.
    pinstate = (((bit_is_set(ENC2_PIN_PORT,ENC2_A)>>ENC2_A)&1)<<1)|((bit_is_set(ENC2_PIN_PORT,ENC2_B)>>ENC2_B)&1);

    state_2 = ttable[state_2 & 0xF][pinstate];

    return (state_2 & 0x30);
}

void checkV (void){

    uint8_t x;

    x = rotary_process_1();

    if (x==0x10) {
        Vvalue += volt10x;
        now=tick;
    };

    if (x==0x20) {
        Vvalue -= volt10x;
        now=tick;
    };

    if (Vvalue >= MAXVOLTS) Vvalue = MAXVOLTS;

    if (Vvalue <= 0) Vvalue = 0;
}


void checkA (void){

    uint8_t x;

    x = rotary_process_2();

    if (x==0x10) {
        Avalue += amps10x;
        now=tick;
    };

    if (x==0x20) {
        Avalue -= amps10x;
        now=tick;
    };

    if (Avalue >= MAXAMPS) Avalue = MAXAMPS;

    if (Avalue <= 0) Avalue = 0;
}


int crc16(const uint8_t* buf, int len){

    unsigned int crc = 0xFFFF;
    int j;
    int i;
    uint8_t b;

    for (j = 0; j <= len-1; j++) {
        b = buf[j];
        for (i = 0; i <= 7; i++) {
            if (((b ^ (uint8_t)crc) & 1)==1) {
                crc=((crc >> 1) ^ 0xA001);
            } else {
                crc=crc>>1;
            };
            b = b>>1;
        };
    };

    return crc;
}


void millis_init(void){

    TIMSK |= (1 << TOIE0);           // Enable overflow Interrupt for Timer/Counter0.
    TCNT0 = 131;                     // Preload Timer with the calculated value for 1 msec.
    TCCR0B |= (1<<CS00)|(1<<CS01);   // Start Timer/Counter0 with Prescaler 64. 8 MHz clock.
}

// Interrupt every 1 msec. More than enough to read the encoders.
ISR(TIMER0_OVF_vect){

    checkV();
    checkA();

    tick++;

    TCNT0 += 131;  // Preload Timer with the calculated value for 1 msec.
}
