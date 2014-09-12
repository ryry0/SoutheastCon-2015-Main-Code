#include "getch.h"
#include "serialnix.h"
#include <stdio.h>

char * port_name = "/dev/ttyACM0";
int main(int argc, char ** argv)
{
  char output = 0;
  int serial_port;
  if (argc > 1)
    port_name = argv[1];

  serial_port = open_port(port_name);
  if (init_serial_port(serial_port) == -1)
    return 1;

  while (1)
  {
    read(serial_port, (char *) &output, 1);
    printf("%c", output);
  }
  return 0;
}
