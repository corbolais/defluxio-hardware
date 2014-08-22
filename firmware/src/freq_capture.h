#ifndef FREQ_CAPTURE_H
#define FREQ_CAPTURE_H 1

// maintain the state of a measurement
typedef enum {
  IDLE,
  PENDING,
  RUNNING
} mstate_t;

void freq_capture_setup(void);
mstate_t freq_get_state(void);
void freq_capture_start(void);
double freq_get_result(void);

#endif /* FREQ_CAPTURE_H */

