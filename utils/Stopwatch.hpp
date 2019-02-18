#pragma once

#include <chrono>

/**
 * Stopwatch implementation to measure elapsed time.
 */
class Stopwatch {
  typedef std::chrono::high_resolution_clock clock_t;

 public:
  /**
   * Starts the timer.
   */
  void start() { start_ = clock_t::now(); }

  /**
   * Stops the timer.
   * @return Elapsed time in seconds.
   */
  double stop() {
    const auto end = clock_t::now();
    elapsed_ =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
            .count() /
        1e6;
    return elapsed_;
  }

  /**
   * Pauses the timer.
   * @return Elapsed time in seconds.
   */
  double pause() {
    const auto end = clock_t::now();
    elapsed_ +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
            .count() /
        1e6;
    return elapsed_;
  }

  /**
   * Elapsed time in seconds.
   */
  double elapsed() const { return elapsed_; }

 protected:
  double elapsed_{0};

 private:
  std::chrono::time_point<clock_t> start_;
};
