#ifndef LOCKOUT_HPP
#define LOCKOUT_HPP

#include <ros/ros.h>
#include <stdint.h>

#include <chrono>

#define schr std::chrono

typedef enum {
  LOC_FREE = 1,
  LOC_ENTER_LOCKED__HOLD = 2,
  LOC_ENTER_LOCKED__RELEASE = -2,
  LOC_LOCKED = -1,
  LOC_EXIT_LOCKED__HOLD = -3,
  LOC_EXIT_LOCKED__RELEASE = 3,
} LockoutState_t;

class LockoutStateMachine {
 private:
  bool *btn1, *btn2;
  LockoutState_t state;
  struct timespec time;
  double stoptime, hold_time;

 public:
  LockoutStateMachine(bool *btn1, bool *btn2, float hold_time_s = 3) {
    this->btn1 = btn1;
    this->btn2 = btn2;
    this->state = LOC_LOCKED;
    this->hold_time = hold_time_s;
  }
  bool isLocked() {
    static double now;
    schr::milliseconds ms = schr::duration_cast<schr::milliseconds>(
        schr::system_clock::now().time_since_epoch());
    now = double(ms.count()) / 1000.0;

    switch (state) {
      case LOC_FREE:
        if (*btn1 && *btn2) {
          state = LOC_ENTER_LOCKED__HOLD;
          stoptime = now + 0.1;  // Give a little time for debounce
        }
        break;
      case LOC_ENTER_LOCKED__HOLD:
        if (!(*btn1 && *btn2)) {
          state = LOC_FREE;
        }
        if (now >= stoptime) {
          state = LOC_ENTER_LOCKED__RELEASE;
        }
        break;
      case LOC_ENTER_LOCKED__RELEASE:
        if (!(*btn1 && *btn2)) {
          state = LOC_LOCKED;
        }
        break;
      case LOC_LOCKED:
        if (*btn1 && *btn2) {
          state = LOC_EXIT_LOCKED__HOLD;
          stoptime = now + hold_time;
        }
        break;
      case LOC_EXIT_LOCKED__HOLD:
        if (!(*btn1 && *btn2)) {
          state = LOC_LOCKED;
          break;
        }
        if (now >= stoptime) {
          state = LOC_EXIT_LOCKED__RELEASE;
        }
        break;

      case LOC_EXIT_LOCKED__RELEASE:
        if (!(*btn1 && *btn2)) {
          state = LOC_FREE;
        }
        break;
      default:
        assert(false);
        break;
    }
    return state < 0;
  }
};

#endif  // End of include guard for LOCKOUT_HPP
