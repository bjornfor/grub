#include <stdio.h>

#include "xhci.h"

int main()
{
  struct xhci *xhci;

  xhci = xhci_create((void*)0x1000, 0);
  xhci_destroy(xhci);

  return 0;
}

