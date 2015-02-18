#ifndef COBS_H_
#define COBS_H_

#include <string.h>
//on non-embedded systems, should return size_t, and should be size_t length.
//Same thing applies to cobes decode
//for destination, pass in an array that is 2 bytes larger than the source
//length.
void cobsEncode(char * source, int length, char * destination) {
  int zero_byte_pos = 1;
  destination [0] = 0;

  //copy source to destination offset by 2 bytes
  memcpy(destination + 2, source, length);
  for (int i = 2; i < length + 2; ++i) {
    if (destination[i] == 0) {
      destination[zero_byte_pos] = i - zero_byte_pos;
      zero_byte_pos = i;
    }
  }
  destination[zero_byte_pos] = length + 2 - zero_byte_pos;
}

void cobsDecode(char * source, int length, char * destination) {
}
#endif
