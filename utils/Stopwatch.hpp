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
  void start() {
    start_ = clock_t::now();
    running_ = true;
  }

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
    running_ = false;
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
    running_ = false;
    return elapsed_;
  }

  /**
   * Elapsed time in seconds.
   */
  double elapsed() const {
    if (!running_)
      return elapsed_;
    else {
      const auto end = clock_t::now();
      return std::chrono::duration_cast<std::chrono::microseconds>(end - start_)
                 .count() /
             1e6;
    }
  }

 protected:
  double elapsed_{0};
  bool running_{false};

 private:
  std::chrono::time_point<clock_t> start_;
};
