/** \file
 *
 *  \brief MOS 6551 Asynchronous Communication Interface Adapter.
 *
 *  \copyright Copyright 2024 Ciaran Anscomb
 *
 *  \licenseblock This file is part of XRoar, a Dragon/Tandy CoCo emulator.
 *
 *  XRoar is free software; you can redistribute it and/or modify it under the
 *  terms of the GNU General Public License as published by the Free Software
 *  Foundation, either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  See COPYING.GPL for redistribution conditions.
 *
 *  \endlicenseblock
 */

// Provides a very simple Linux implementation, intended for using two FIFO
// files (one for TX and one for RX)
//
// Support for RX interrupts is implemented (not TX)
//

#include "top-config.h"

#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pwd.h>
#include <unistd.h>
#include <alloca.h>

#include "array.h"

#include "logging.h"
#include "mos6551.h"
#include "part.h"
#include "serialise.h"
#include "xroar.h"

// register bits
#define STAT_REG_RX_FULL (1<<3)
#define STAT_REG_TX_FULL (1<<4)
#define STAT_REG_IRQ (1<<7)

#define CMD_REG_RX_IRQ_EN (1<<1)
#define CMD_REG_DTR (1<<0)

static const struct ser_struct ser_struct_mos6551[] = {
	SER_ID_STRUCT_ELEM(1, struct MOS6551, status_reg),
	SER_ID_STRUCT_ELEM(2, struct MOS6551, command_reg),
	SER_ID_STRUCT_ELEM(3, struct MOS6551, control_reg),
};

static const struct ser_struct_data mos6551_ser_struct_data = {
	.elems = ser_struct_mos6551,
	.num_elems = ARRAY_N_ELEMENTS(ser_struct_mos6551),
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// MOS6551 ACIA part creation

static struct part *mos6551_allocate(void);
static _Bool mos6551_finish(struct part *p);
static void mos6551_free(struct part *p) ;
static void do_irq(void *sptr);
static void try_rx(struct MOS6551 *acia);
static void try_tx(struct MOS6551 *acia);

static const struct partdb_entry_funcs mos6551_funcs = {
	.allocate = mos6551_allocate,
	.finish = mos6551_finish,
	.free = mos6551_free,

	.ser_struct_data = &mos6551_ser_struct_data,
};

const struct partdb_entry mos6551_part = { .name = "MOS6551", .description = "MOS Technology | 6551 ACIA", .funcs = &mos6551_funcs };

static struct part *mos6551_allocate(void) {
	struct MOS6551 *acia = part_new(sizeof(*acia));
	struct part *p = &acia->part;
	long buflen = sysconf(_SC_GETPW_R_SIZE_MAX) ;
	char *buf = alloca(buflen) ;
	struct passwd pwd, *p_pwd ;
	char path[256] ;
	
	*acia = (struct MOS6551){0};

	acia->status_reg = STAT_REG_TX_FULL;
    event_init(&acia->irq_event, DELEGATE_AS0(void, do_irq, acia));
    
	// path to home folder
	if (!getpwuid_r(getuid(), &pwd, buf, buflen, &p_pwd ))
	{
		// open the TX & RX FIFOs (if they exist)
		snprintf(path,sizeof(path), "%s/.xroar/tx_uart", pwd.pw_dir) ;
	    acia->fd_tx = open( path, O_RDWR | O_NONBLOCK ) ;
		snprintf(path,sizeof(path), "%s/.xroar/rx_uart", pwd.pw_dir) ;
	    acia->fd_rx = open( path, O_RDONLY | O_NONBLOCK ) ;
	}
	return p;
}

static _Bool mos6551_finish(struct part *p) {
	struct MOS6551 *acia = (struct MOS6551 *)p;
	
	// No-op
	(void)acia;
	
	return 1;
}

static void mos6551_free(struct part *p) {
	struct MOS6551 *acia = (struct MOS6551 *)p;
	
	event_dequeue(&acia->irq_event);
	if(acia->fd_tx>=0 )
	{
		close(acia->fd_tx) ;
		acia->fd_tx = -1 ;
	}
	if(acia->fd_rx>=0 )
	{
		close(acia->fd_rx) ;
		acia->fd_rx = -1 ;
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void mos6551_reset(struct MOS6551 *acia) {
	// W65C51N datasheet says bit 4 of the status register (Transmitter
	// Data Register Empty) is always set.  Not sure if that's common
	// across variants, but I think this bit is also something the Dragon
	// 64 ROM checks for.
	acia->status_reg = STAT_REG_TX_FULL;
	acia->command_reg = 0;
	acia->control_reg = 0;
}

static void try_tx(struct MOS6551 *acia)
{
	// if got data to send...
	if ( acia->status_reg & STAT_REG_TX_FULL )
		return ;

	// ...try and send it, if successful clear the tx-full bit
	if ( acia->fd_tx >= 0 )
	{
		if ( write(acia->fd_tx,&acia->tx_data,sizeof(uint8_t)) == sizeof(uint8_t) )
			acia->status_reg|=STAT_REG_TX_FULL ;
	}
}

static void try_rx(struct MOS6551 *acia)
{
	// if DTR not asserted, bail
	if ( !(acia->command_reg & CMD_REG_DTR) )
		return ;

	// if already got unread data, bail
	if ( acia->status_reg & STAT_REG_RX_FULL )
		return ;

	// read any pending data 
	if ( acia->fd_rx >= 0 )
	{
		if ( read(acia->fd_rx,&acia->rx_data,sizeof(uint8_t)) == sizeof(uint8_t) )
		{
			acia->status_reg|=STAT_REG_RX_FULL ;
			// generate IRQ if enabled.
			if ( !(acia->command_reg & CMD_REG_RX_IRQ_EN) && !event_queued(&acia->irq_event))
			{
				event_queue(&MACHINE_EVENT_LIST, &acia->irq_event);
				acia->status_reg|=STAT_REG_IRQ;
			}
		}
	}		
}

static void mos6551_read(struct MOS6551 *acia, unsigned A, uint8_t *D) {

	switch (A & 3) {
	default:
	case 0:
		// Receive data
		*D = acia->rx_data ;
		acia->status_reg&=~STAT_REG_RX_FULL;
		break;
	case 1:
		// Status register
		*D = acia->status_reg;
		acia->IRQ = 0;
		acia->status_reg &= ~STAT_REG_IRQ;
		break;
	case 2:
		// Command register
		*D = acia->command_reg;
		break;
	case 3:
		// Control register
		*D = acia->control_reg;
		break;
	}
}

static void mos6551_write(struct MOS6551 *acia, unsigned A, uint8_t *D) {
	switch (A & 3) {
	default:
	case 0:
		// Transmit data
		acia->tx_data = *D ;
		acia->status_reg&=~STAT_REG_TX_FULL;
		// try and send it now
		try_tx(acia) ;
		break;
	case 1:
		// Programmed reset
		acia->command_reg &= ~0x1f;
		// NOTE: the W65C51N datasheet claims their part clears
		// this bit on programmed reset (i.e. _enables_ IRQ).
		acia->command_reg |= 0x02;
		break;
	case 2:
		// Command register
		acia->command_reg = *D;
		break;
	case 3:
		// Control register
		acia->control_reg = *D;
		break;
	}
}

void mos6551_access(void *sptr, _Bool RnW, unsigned A, uint8_t *D) {
	struct MOS6551 *acia = sptr;

	if (RnW) {
		mos6551_read(acia, A, D);
	} else {
		mos6551_write(acia, A, D);
	}
}

static void do_irq(void *sptr) {
	struct MOS6551 *acia = sptr;

	acia->IRQ = 1;
}

void mos6551_service_uarts(struct MOS6551 *acia)
{
	try_tx(acia) ;
	try_rx(acia) ;
}
