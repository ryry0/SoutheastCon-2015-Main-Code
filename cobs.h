#ifndef COBS_H_
#define COBS_H_

#include <string.h>
#include <stdlib.h>

/*
   cobs Encode in place does the encoding on the same array passed in, and does
   not require using a different array. This may be preferable for performance
   reasons. Encode in place assumes that the first byte is the header byte and
   the second byte is the code byte.
   */
void cobsEncodeInPlace(char* buffer, const unsigned int length) {
  unsigned int zero_byte_pos = 1;

  for (unsigned int i = 2; i < length; ++i) {
    if (buffer[i] == 0) {
      buffer[zero_byte_pos] = i - zero_byte_pos;
      zero_byte_pos = i;
    }
  }
  buffer[zero_byte_pos] = length - zero_byte_pos;
}

/*
   on non-embedded systems, should return size_t, and should be size_t length.
   Same thing applies to cobes decode.
   For destination, pass in an array that is 2 bytes larger than the source
   length.
   */
void cobsEncode(const char* source, const unsigned int length, char* destination) {
  destination [0] = 0;

  //copy source to destination offset by 2 bytes
  memcpy(destination + 2, source, length);
  cobsEncodeInPlace(destination, length + 2);
}

/*
   Decode in place does the decoding on the same array passed in. Decode in place
   assumes that the first byte is the header byte and the second byte is the code
   byte.
   */
void cobsDecodeInPlace(char* buffer, const unsigned int length) {
  unsigned int zero_byte_pos = buffer[1];
  unsigned int current_zero_byte_pos = 1;

  for (unsigned int i = 2; i < length; ++i) {
    if (i - current_zero_byte_pos == zero_byte_pos) {
      zero_byte_pos = buffer[i];
      current_zero_byte_pos = i;
      buffer[i] = 0;
    }
  }
}

/*
   on non-embedded systems, should return size_t, and should be size_t length.
   Same thing applies to cobes decode.
   For destination, pass in an array that is at least as large as the source
   length minus 2.
   Strips the  header and code byte from the data.
   */
void cobsDecode(const char* source, const unsigned int length, char* destination) {
  char* buffer = (char*) malloc(length);

  memcpy(buffer, source, length);
  cobsDecodeInPlace(buffer, length);
  memcpy(destination,buffer+2, length-2);

  free(buffer);
}
#endif
