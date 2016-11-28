#ifndef XHCI_DEBUG_H
#define XHCI_DEBUG_H

struct xhci;

int xhci_debug_enabled(void);

void xhci_trace (const char *fmt, ...);

void xhci_dbg (const char *fmt, ...);

void xhci_err (const char *fmt, ...);

int xhci_dump_oper(struct xhci *xhci);

void xhci_dump_oper_portsc(struct xhci *xhci, int port);

int xhci_dump_cap(struct xhci *xhci);

#endif /* XHCI_DEBUG_H */
