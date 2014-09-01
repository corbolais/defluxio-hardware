#include "log_formatter.h"
#include <stdio.h>

void log_info(char* msg) {
  printf("I;%s\n", msg);
  fflush(stdout);
}

void log_freq(double freq) {
  printf("F;%7.5f\n", freq);
  fflush(stdout);
}
