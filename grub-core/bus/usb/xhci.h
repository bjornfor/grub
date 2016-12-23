/*
 * This file is part of the libpayload project.
 *
 * Copyright (C) 2010 Patrick Georgi
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef __XHCI_H
#define __XHCI_H

//#include <pci.h>
//#include <usb/usb.h>

#include <stdint.h>

typedef enum {
	FULL_SPEED = 0, LOW_SPEED = 1, HIGH_SPEED = 2, SUPER_SPEED = 3,
} usb_speed;

typedef enum { SETUP, IN, OUT } direction_t;
typedef enum { CONTROL = 0, ISOCHRONOUS = 1, BULK = 2, INTERRUPT = 3
} endpoint_type;

typedef struct xhci xhci_t;

void xhci_start (xhci_t *const xhci);
void xhci_stop (xhci_t *const xhci);
void xhci_reset (xhci_t *const xhci);
void xhci_reinit (xhci_t *const xhci);
void xhci_shutdown (xhci_t *const xhci);

//xhci_t *xhci_pci_init (pcidev_t addr);
xhci_t *xhci_init (unsigned long physical_bar);

//void xhci_rh_init (usbdev_t *dev);

int xhci_num_ports(xhci_t *const xhci);

int
xhci_rh_hub_status_changed(xhci_t *const xhci);

int
xhci_rh_port_status_changed(xhci_t *const xhci, const int port);

int
xhci_rh_port_connected(xhci_t *const xhci, const int port);

int
xhci_rh_port_in_reset(xhci_t *const xhci, const int port);

int
xhci_rh_port_enabled(xhci_t *const xhci, const int port);

usb_speed
xhci_rh_port_speed(xhci_t *const xhci, const int port);

int
xhci_rh_attach_dev(xhci_t *xhci, const int port);

int
xhci_rh_reset_port(xhci_t *const xhci, const int port);

int
xhci_rh_enable_port(xhci_t *const xhci, int port);

void
xhci_rh_init (xhci_t *const xhci);

int
xhci_rh_hub_scanport(xhci_t *xhci, const int port);

void
xhci_rh_hub_poll(xhci_t *const xhci);

#endif
