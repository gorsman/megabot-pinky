#ifndef _MegaBot_debug_h_
#define _MegaBot_debug_h_

//
// Declare this define before in the file you want to debug.
// You must to that before including this header file.
// Example <code_to_be_debugged.cpp>:
//   line 0: #define DEBUG
//   line 1: #include "debug.h"
//
// #define DEBUG

#ifdef DEBUG

#define DEBUG_PRINT(out0) { \
  Serial.println(out0); \
}

#define DEBUG_PRINT2(out0, out1) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.println(out1); \
}

#define DEBUG_PRINT3(out0, out1, out2) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.print(out1); Serial.print(" "); \
  Serial.println(out2); \
}

#define DEBUG_PRINT4(out0, out1, out2, out3) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.print(out1); Serial.print(" "); \
  Serial.print(out2); Serial.print(" "); \
  Serial.println(out3); \
}

#define DEBUG_PRINT5(out0, out1, out2, out3, out4) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.print(out1); Serial.print(" "); \
  Serial.print(out2); Serial.print(" "); \
  Serial.print(out3); Serial.print(" "); \
  Serial.println(out4); \
}

#define DEBUG_PRINT6(out0, out1, out2, out3, out4, out5) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.print(out1); Serial.print(" "); \
  Serial.print(out2); Serial.print(" "); \
  Serial.print(out3); Serial.print(" "); \
  Serial.print(out4); Serial.print(" "); \
  Serial.println(out5); \
}

#define DEBUG_PRINT7(out0, out1, out2, out3, out4, out5, out6) { \
  Serial.print(out0); Serial.print(" "); \
  Serial.print(out1); Serial.print(" "); \
  Serial.print(out2); Serial.print(" "); \
  Serial.print(out3); Serial.print(" "); \
  Serial.print(out4); Serial.print(" "); \
  Serial.print(out5); Serial.print(" "); \
  Serial.println(out6); \
}

#define DEBUG_THROTTLE_START(delay) { \
  static long last_exec_time = 0; \
  long cur_time = millis(); \
  if (last_exec_time + delay < cur_time) { {

#define DEBUG_THROTTLE_END() } last_exec_time = cur_time; } }


#else

#define DEBUG_PRINT(out0) ;
#define DEBUG_PRINT2(out0, out1) ;
#define DEBUG_PRINT3(out0, out1, out2) ;
#define DEBUG_PRINT4(out0, out1, out2, out3) ;
#define DEBUG_PRINT5(out0, out1, out2, out3, out4) ;
#define DEBUG_PRINT6(out0, out1, out2, out3, out4, out5) ;
#define DEBUG_PRINT7(out0, out1, out2, out3, out4, out5, out6) ;

#define DEBUG_THROTTLE_START(delay) { if (false) {
#define DEBUG_THROTTLE_END() } }

#endif



// Digital pin that is attached to the LED that will blink to indicate
// fatal error. Comment this #define if you don't want any LED blicking
// on fatal error.
#define FATAL_ERROR_INDICATOR_PIN 13

#define FATAL_ERROR(msg) { debug::FatalError(msg); }

namespace debug {

void FatalError(char* msg);

}  // namespace debug

#endif  // #ifndef _MegaBot_debug_h_ 
