#include <Arduino.h>
#include "debug.h"

namespace debug {

void FatalError(char* msg) {
  Serial.print("FATAL ERROR: ");
  Serial.println(msg);
#ifdef FATAL_ERROR_INDICATOR_PIN
  pinMode(FATAL_ERROR_INDICATOR_PIN, OUTPUT);
  while (1) {
    digitalWrite(FATAL_ERROR_INDICATOR_PIN, HIGH);
    delay(500);
    digitalWrite(FATAL_ERROR_INDICATOR_PIN, LOW);
    delay(500);
  };
#else
  while (1) { delay(1000); };
#endif
}

}  // namespace debug
