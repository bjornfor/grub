/*
 * This file is part of the libpayload project.
 *
 * Copyright (C) 2013 secunet Security Networks AG
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

//#define USB_DEBUG

//#include <usb/usb.h>
//#include "generic_hub.h"
#include "xhci_private.h"
#include "xhci_io.h"
#include "xhci.h"

int
xhci_rh_hub_status_changed(xhci_t *const xhci)
{
	const int changed = !!(xhci->opreg->usbsts & USBSTS_PCD);
	if (changed)
		xhci->opreg->usbsts =
			(xhci->opreg->usbsts & USBSTS_PRSRV_MASK) | USBSTS_PCD;
	return changed;
}

int
xhci_rh_port_status_changed(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	const int changed = !!(*portsc & (PORTSC_CSC | PORTSC_PRC));
	/* always clear all the status change bits */
	*portsc = (*portsc & PORTSC_RW_MASK) | 0x00fe0000;
	return changed;
}

int
xhci_rh_port_connected(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	return *portsc & PORTSC_CCS;
}

int
xhci_rh_port_in_reset(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	return !!(*portsc & PORTSC_PR);
}

int
xhci_rh_port_enabled(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	return !!(*portsc & PORTSC_PED);
}

usb_speed
xhci_rh_port_speed(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	if (*portsc & PORTSC_PED) {
		return ((*portsc & PORTSC_PORT_SPEED_MASK)
				>> PORTSC_PORT_SPEED_START)
			- 1;
	} else {
		return -1;
	}
}

static int
wait_for_port(xhci_t *const xhci, const int port,
			  const int wait_for,
			  int (*const port_op)(xhci_t *, int),
			  int timeout_steps, const int step_ms)
{
	int state;
	do {
		state = port_op(xhci, port);
		if (state < 0)
			return -1;
		else if (!!state == wait_for)
			return timeout_steps;
		xhci_mdelay(step_ms);
		--timeout_steps;
	} while (timeout_steps);
	return 0;
}

int
xhci_rh_reset_port(xhci_t *const xhci, const int port)
{
	volatile u32 *const portsc = &xhci->opreg->prs[port - 1].portsc;

	/* Trigger port reset. */
	*portsc = (*portsc & PORTSC_RW_MASK) | PORTSC_PR;

	/* Wait for port_in_reset == 0, up to 150 * 1000us = 150ms */
	if (wait_for_port(xhci, port, 0, xhci_rh_port_in_reset,
				      150, 1000) == 0)
		xhci_debug("xhci_rh: Reset timed out at port %d\n", port);
	else
		/* Clear reset status bits, since port is out of reset. */
		*portsc = (*portsc & PORTSC_RW_MASK) | PORTSC_PRC | PORTSC_WRC;

	return 0;
}

int
xhci_rh_enable_port(xhci_t *const xhci, int port)
{
	if (1 /*IS_ENABLED(CONFIG_LP_USB_XHCI_MTK_QUIRK)*/) {
		volatile u32 *const portsc =
			&xhci->opreg->prs[port - 1].portsc;

		/*
		 * Before sending commands to a port, the Port Power in
		 * PORTSC register should be enabled on MTK's xHCI.
		 */
		*portsc = (*portsc & PORTSC_RW_MASK) | PORTSC_PP;
	}
	return 0;
}

void
xhci_rh_init (xhci_t *const xhci)
{
	const int num_ports = /* TODO: maybe we need to read extended caps */
		(xhci->capreg->hcsparams1 >> 24) & 0xff;

	xhci_debug("xHCI: root hub init done\n");
}
