/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://tinyurl.com/peterfleury
File:     $Id: uart.c,v 1.15.2.4 2015/09/05 18:33:32 peter Exp $
Software: AVR-GCC 4.x
Hardware: any AVR with built-in UART,
License:  GNU General Public License

DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.

    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a
    power of 2.

USAGE:
    Refere to the header file uart.h for a description of the routines.
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306

LICENSE:
    Copyright (C) 2015 Peter Fleury, GNU General Public License Version 3

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"


/*
 *  constants and macros
 */

/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

 #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
 #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
 #define UART0_STATUS      UCSRA
 #define UART0_CONTROL     UCSRB
 #define UART0_CONTROLC    UCSRC
 #define UART0_DATA        UDR
 #define UART0_UDRIE       UDRIE
 #define UART0_UBRRL       UBRRL
 #define UART0_UBRRH       UBRRH
 #define UART0_BIT_U2X     U2X
 #define UART0_BIT_RXCIE   RXCIE
 #define UART0_BIT_RXEN    RXEN
 #define UART0_BIT_TXEN    TXEN
 #define UART0_BIT_UCSZ0   UCSZ0
 #define UART0_BIT_UCSZ1   UCSZ1


/*
 *  module global variables
 */
static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static volatile unsigned char UART_LastRxError;


ISR (UART0_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;


    /* read UART status register and UART data register */
    usr  = UART0_STATUS;
    data = UART0_DATA;

    /* get FEn (Frame Error) DORn (Data OverRun) UPEn (USART Parity Error) bits */
#if defined(FE) && defined(DOR) && defined(UPE)
    lastRxError = usr & (_BV(FE)|_BV(DOR)|_BV(UPE) );
#elif defined(FE0) && defined(DOR0) && defined(UPE0)
    lastRxError = usr & (_BV(FE0)|_BV(DOR0)|_BV(UPE0) );
#elif defined(FE1) && defined(DOR1) && defined(UPE1)
    lastRxError = usr & (_BV(FE1)|_BV(DOR1)|_BV(UPE1) );
#elif defined(FE) && defined(DOR)
    lastRxError = usr & (_BV(FE)|_BV(DOR) );
#endif

    /* calculate buffer index */
    tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;

    if ( tmphead == UART_RxTail ) {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }else{
        /* store new index */
        UART_RxHead = tmphead;
        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError |= lastRxError;
}


ISR (UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;


    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    }else{
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}


/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(unsigned int baudrate)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

#ifdef UART_TEST
#ifndef UART0_BIT_U2X
#warning "UART0_BIT_U2X not defined"
#endif
#ifndef UART0_UBRRH
#warning "UART0_UBRRH not defined"
#endif
#ifndef UART0_CONTROLC
#warning "UART0_CONTROLC not defined"
#endif
#if defined(URSEL) || defined(URSEL0)
#ifndef UART0_BIT_URSEL
#warning "UART0_BIT_URSEL not defined"
#endif
#endif
#endif

    /* Set baud rate */
    if ( baudrate & 0x8000 )
    {
        #if UART0_BIT_U2X
        UART0_STATUS = (1<<UART0_BIT_U2X);  //Enable 2x speed
        #endif
    }
    #if defined(UART0_UBRRH)
    UART0_UBRRH = (unsigned char)((baudrate>>8)&0x80) ;
    #endif
    UART0_UBRRL = (unsigned char) (baudrate&0x00FF);

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(UART0_BIT_RXCIE)|(1<<UART0_BIT_RXEN)|(1<<UART0_BIT_TXEN);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    #ifdef UART0_CONTROLC
    #ifdef UART0_BIT_URSEL
    UART0_CONTROLC = (1<<UART0_BIT_URSEL)|(1<<UART0_BIT_UCSZ1)|(1<<UART0_BIT_UCSZ0);
    #else
    UART0_CONTROLC = (1<<UART0_BIT_UCSZ1)|(1<<UART0_BIT_UCSZ0);
    #endif
    #endif

}/* uart_init */

/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting the receive buffer.  Acutally ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart_flush(void)
{
        UART_RxHead = UART_RxTail;
}/* uart_flush */

/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart_getc(void)
{
    unsigned char tmptail;
    unsigned char data;
    unsigned char lastRxError;


    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }

    /* calculate buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;

    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];
    lastRxError = UART_LastRxError;

    /* store buffer index */
    UART_RxTail = tmptail;

    UART_LastRxError = 0;
    return (lastRxError << 8) + data;

}/* uart_getc */


/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(unsigned char data)
{
    unsigned char tmphead;


    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    while ( tmphead == UART_TxTail ){
        ;/* wait for free space in buffer */
    }

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART0_CONTROL    |= _BV(UART0_UDRIE);

}/* uart_putc */
