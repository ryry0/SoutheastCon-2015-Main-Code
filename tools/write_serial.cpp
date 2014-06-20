#include "getch.h"
#include "serialnix.h"
#include <stdio.h>

char * port_name = "/dev/ttyACM0";

int main()
{
  char input = 0;
  char output = 0;
  int serial_port = open_port(port_name);
  init_serial_port(serial_port);

  while (1)
  {
    input = getch();

    if (input != 0)
      write(serial_port, (char *) &input, 1);
  }
  return 0;
}
