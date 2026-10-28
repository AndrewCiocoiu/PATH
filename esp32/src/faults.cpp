#include "faults.h"

#include <string.h>

static FaultEvent queue[16];
static uint8_t head = 0, count = 0;

void fault_report(uint16_t code, uint8_t severity, bool active, const char *text) {
  if (count == 16) {  // full: drop the oldest
    head = (head + 1) % 16;
    count--;
  }
  FaultEvent &f = queue[(head + count) % 16];
  f.code = code;
  f.severity = severity;
  f.active = active;
  strncpy(f.text, text, sizeof(f.text) - 1);
  f.text[sizeof(f.text) - 1] = '\0';
  count++;
}

bool fault_pop(FaultEvent &out) {
  if (count == 0) return false;
  out = queue[head];
  head = (head + 1) % 16;
  count--;
  return true;
}
