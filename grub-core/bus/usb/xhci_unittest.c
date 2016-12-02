#include <stdio.h>

#include "xhci.h"

int main()
{
  struct xhci *xhci;

  for (int i=0; i<10; i++)
  {
    xhci = xhci_create((void*)0x1000, 0);
    xhci_destroy(xhci);
  }

  return 0;
}

