#include <Arduino.h>
#include "scheduling.h"

// #define DEBUG
#include "debug.h"

// In order to actually see profiling info you need to uncomment #define DEBUG
#define PROFILING_INFO_OUTPUT_PERIOD_MILLIS 5000

namespace scheduling {

Task::Task() : state_(0), next_run_time_millis_(0L) { }

bool Task::Loop(long time_millis) {
  if (time_millis < next_run_time_millis_) return false;
  Run();
  return true;
}

void Task::Run() {
  FATAL_ERROR("Pure virtual Task::Run is called");
}

void Task::Delay(long delay_millis) {
  next_run_time_millis_ =
      max(next_run_time_millis_, millis() + delay_millis);
}


Scheduler::Scheduler() {
  memset(tasks_, 0, sizeof(tasks_));
}

void Scheduler::AddTask(char* name, Task* task) {
  for (int i = 1; i < MAX_SCHEDULER_TASKS; ++i) {
    if (!tasks_[i].task) {
      tasks_[i].name = name;
      tasks_[i].task = task;
      tasks_[i].time_stat.Clear();
      DEBUG_PRINT4("Added task <", name, "> #", i);
      return;
    }
  }
  FATAL_ERROR("Exceeded max scheduler tasks");
}

void Scheduler::Loop() {
  long cur_time = millis();
  for (int i = 0; i < MAX_SCHEDULER_TASKS; ++i) {
    if (tasks_[i].task) {
      Task* task = tasks_[i].task;
      TICK();
      bool executed = task->Loop(cur_time);
      if (executed) {
        // Since the task was actually executed we update current time.
        cur_time = millis();
      	long time_executed = TICK();
        if (time_executed > 0) {
          tasks_[i].time_stat.Record(time_executed);
        }
      }
    }
  }
  
  DEBUG_THROTTLE_START(PROFILING_INFO_OUTPUT_PERIOD_MILLIS);
  DEBUG_PRINT(">>> Scheduler profiling info");
  for (int i = 0; i < MAX_SCHEDULER_TASKS; ++i) {
    if (tasks_[i].name) {
      VALUE_STAT_PRINT(tasks_[i].name, tasks_[i].time_stat);
    }
  } 
  DEBUG_PRINT("<<<");
  DEBUG_THROTTLE_END();
}

}  // namespace scheduling
