/*
  Author:Ryan - David Reyes
*/
#ifndef SERIALNIX_H
#define SERIALNIX_H

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


int open_port(char * port_name)//returns the file descriptor or -1;
{
  int fd; //file descriptor for the port
  fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd==-1)
  {
    perror("open_port: Unable to open serial port");
  }
  else
    fcntl(fd, F_SETFL, 0);

  return fd;

}

int init_serial_port(int fd) //returns -1 if not successful
{
  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, B9600);
  cfsetospeed(&options, B9600);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;

  return tcsetattr(fd, TCSANOW, &options);
}

#endif
