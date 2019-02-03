#pragma once

#include <chrono>

/**
 * Stopwatch implementation to measure elapsed time.
 */
class Stopwatch {
 public:
  typedef std::chrono::high_resolution_clock clock_t;

  /**
   * Starts the timer.
   */
  virtual void start() {
    _begin = clock_t::now();
    time = 0;
  }

  /**
   * Stops the timer.
   * @return Elapsed time in seconds.
   */
  virtual double stop() {
    auto _end = clock_t::now();
    time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin)
                .count() /
            1e3;
    return time;
  }

  /**
   * Pauses the timer. Has the same functionality as Stopwatch::stop().
   * @return Elapsed time in seconds.
   */
  virtual double pause() {
    auto _end = clock_t::now();
    time += std::chrono::duration_cast<std::chrono::milliseconds>(_end - _begin)
                .count() /
            1e3;
    return time;
  }

  /**
   * Resumes the timer without resetting the time.
   */
  virtual void resume() { _begin = clock_t::now(); }

  /**
   * Elapsed time in seconds.
   */
  double time{0};

 private:
  std::chrono::time_point<clock_t> _begin;
};
