#include "getch.h"
#include "serialnix.h"
#include <iostream>

int main()
{
  char input = 0;
  int serial_port = open_port();
  init_serial_port(serial_port);

  while (1)
  {
    input = getch();

    if (input != 0)
      write(serial_port, (char *) &input, 1);
  }
  return 0;
}
