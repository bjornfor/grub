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

static int
hub_debounce(xhci_t *xhci, const int port)
{
	const int step_ms	= 1;	/* linux uses 25ms, we're busy anyway */
	const int at_least_ms	= 100;	/* 100ms as in usb20 spec 9.1.2 */
	const int timeout_ms	= 1500;	/* linux uses this value */

	int total_ms = 0;
	int stable_ms = 0;
	while (stable_ms < at_least_ms && total_ms < timeout_ms) {
		xhci_mdelay(step_ms);

		const int changed = xhci_rh_port_status_changed(xhci, port);
		const int connected = xhci_rh_port_connected(xhci, port);
		if (changed < 0 || connected < 0)
			return -1;

		if (!changed && connected) {
			stable_ms += step_ms;
		} else {
			xhci_debug("generic_hub: Unstable connection at %d\n",
					port);
			stable_ms = 0;
		}
		total_ms += step_ms;
	}
	if (total_ms >= timeout_ms)
		xhci_debug("generic_hub: Debouncing timed out at %d\n", port);
	return 0; /* ignore timeouts, try to always go on */
}

int
xhci_rh_attach_dev(xhci_t *xhci, const int port)
{
	if (hub_debounce(xhci, port) < 0)
		return -1;

	if (xhci_rh_reset_port(xhci, port) < 0)
		return -1;
	/* after reset the port will be enabled automatically */
	const int ret = wait_for_port(
			/* time out after 1,000 * 10us = 10ms */
			xhci, port, 1, xhci_rh_port_enabled, 1000, 10);
	if (ret < 0)
		return -1;
	else if (!ret)
		xhci_debug("generic_hub: Port %d still "
				"disabled after 10ms\n", port);

	const usb_speed speed = xhci_rh_port_speed(xhci, port);
	if (speed >= 0) {
		xhci_debug("generic_hub: Success at port %d\n", port);
		xhci_mdelay(10); /* Reset recovery time
			       (usb20 spec 7.1.7.5) */
	}
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
	(void)xhci;
	//const int num_ports = /* TODO: maybe we need to read extended caps */
		//(xhci->capreg->hcsparams1 >> 24) & 0xff;

	xhci_debug("xHCI: root hub init done\n");
}

int
xhci_rh_hub_scanport(xhci_t *xhci, const int port)
{
	if (xhci_rh_port_connected(xhci, port)) {
		xhci_debug("generic_hub: Attachment at port %d\n", port);

		return xhci_rh_attach_dev(xhci, port);
	} else {
		xhci_debug("generic_hub: Detachment at port %d\n", port);
	}

	return 0;
}

void
xhci_rh_hub_poll(xhci_t *const xhci)
{
	if (xhci_rh_hub_status_changed(xhci) != 1)
		return;

	int port;
	int num_ports = (xhci->capreg->hcsparams1 >> 24) & 0xff;
	for (port = 1; port <= num_ports; ++port) {
		const int ret = xhci_rh_port_status_changed(xhci, port);
		if (ret < 0) {
			return;
		} else if (ret == 1) {
			xhci_debug("generic_hub: Port change at %d\n", port);
			if (xhci_rh_hub_scanport(xhci, port) < 0)
				return;
		}
	}
}
