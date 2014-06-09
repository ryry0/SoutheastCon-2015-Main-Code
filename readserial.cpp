#include "getch.h"
#include "serialnix.h"
#include <stdio.h>

int main()
{
  char output = 0;
  int serial_port = open_port();
  init_serial_port(serial_port);

  while (1)
  {
    read(serial_port, (char *) &output, 1);
    printf("%c", output);
  }
  return 0;
}
