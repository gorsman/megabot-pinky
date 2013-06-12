#ifndef _MegaBot_scheduling_h_
#define _MegaBot_scheduling_h_

#include <inttypes.h>

#include "profiling.h"

namespace scheduling {

#define MAX_SCHEDULER_TASKS 32

#define STATE_MAP_START { switch(state_) {
#define STATE_START(STATE) case STATE: {
#define STATE_END } break;
#define STATE_MAP_END default: Delay(10000); return; } }

#define STATE_TRANSITION(NEXT_STATE) { state_ = NEXT_STATE; return; }
#define STATE_TRANSITION_AFTER(NEXT_STATE, DELAY) { Delay(DELAY); STATE_TRANSITION(NEXT_STATE); }

// Task is an abstraction that represents concurrently executing tasks
// of the application. Scheduler is a very basic implementation of multi-tasking
// mechanism for Arduino.
// In order to unblock Arduino application from executing other tasks, no task
// should ever do blocking calls. In particular the tasks should never call methods
// like 'delay' - instead a 'Delay' method is provided, which essentially sets a
// timeout that determines when the task executes next time.
//
// Note: in reality implementation of completely non-blocking tasks is not possible.
// For instance, the tasks that do any sort of serial communication can only do it as fast
// as communication channel allows (e.g. 100kHz communication channel only transfers ~100
// bits per ms). So the rule of thumb is: the task's Run method should return as quick as
// possible. If task's Run method executes within 100 microseconds - that's probably fine,
// if it takes several ms - that might be unacceptable (as some other tasks might not be
// able to execute properly). 
//
// To implement your Task you need to override 'Run' method. Basic Task implementation
// example looks like this:
//
//   class MyTask : public scheduling::Task {
//     public:
//       virtual void Run() {
//         // do something
//         
//         if (we_need_to_wait) {
//           Delay(1000);  // we wait for 1000ms
//           return;       // 'Delay' doesn't return from 'Run' automatically
//         }
//       }
//   };
//
// Often times you need to do different things on different consecutive calls of Run method.
// For instance, imagine you're controlling a range sensor. With the first call of 'Run' method
// you issue a command to the sensor to send an impulse. Then you have to wait for ~70-100ms
// for the impulse to bounce back (that is in case of ultrasonic range sensor). After the delay
// you need to read the value measured by the sensor.
// To do this kind of things it's convenient to turn task into a state machine. Here is an example
// of how you can do that using the STATE_* macroses provided in this file:
//
//   class RangeSensorTask : public scheduling::Task {
//      public:
//        enum State {
//          INITIAL,  // state 0 is initial state of the Task by convention
//          SEND_IMPULSE,
//          READ_IMPULSE
//        };
//
//        virtual void Run() {
//          STATE_MAP_START
//      
//            STATE_START(INITIAL)
//              // some other code might be here
//              STATE_TRANSITION(SEND_IMPULSE);
//            STATE_END;
//        
//            STATE_START(SEND_IMPULSE)
//              range_sensor_.SendImpulse();
//              STATE_TRANSITION_AFTER(READ_IMPULSE, 100);
//            STATE_END;
//        
//            STATE_START(READ_IMPULSE)
//              System.println(range_sensor_.ReadImpulse());
//              // let's wait 900ms now so that we get one measurement every second
//              STATE_TRANSITION_AFTER(INITIAL, 900);
//            STATE_END;
//        
//          STATE_MAP_END;
//        }
//      private:
//        RangeSensor range_sensor_;
//   };
class Task {
  public:
    Task();

    // This method is going to be repeatedly called by the Scheduler.
    // Returns whether the task was actualy run.
    // Clients must not override this - instead override Run method.
    bool Loop(long time_millis);

    // This method should be implemented by subclasses.
    virtual void Run();

    void Delay(long delay_millis);

  protected:
    int8_t state_;
    long next_run_time_millis_;     
};

class Scheduler {
  public:
    Scheduler();

    // Doesn't take ownership of any of the arguments.
    void AddTask(char* name, Task* task);
    void Loop();
  
  private:
    struct Task_ {
      char* name;
      Task* task;
      profiling::ValueStat time_stat;
      Task_() : name(0), task(0), time_stat() { } 
    };
    Task_ tasks_[MAX_SCHEDULER_TASKS];
};


}  // namespace scheduling

#endif  // #ifndef _MegaBot_scheduling_h_
