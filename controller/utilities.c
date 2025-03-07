#include "utilities.h"

volatile enum mode mode = IDLE;

void set_mode(enum mode m) {
  mode = m;
}

enum mode get_mode() {
  return mode;
}