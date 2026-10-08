#pragma once
#include <cmath>
#include <cstdint>
#include <ostream>
#include <stdexcept>

// Logging-only identity: never use these checks to change integration endpoints.
class LtvEventReceipt {
public:
  uint64_t id = 0;
  int64_t camera_ns = 0, right_camera_ns = 0;
  double physical_camera_time = 0;

  void begin(int64_t left, int64_t right, double time) {
    if (pending_)
      throw std::runtime_error("Previous camera receipt was not logged");
    if (!std::isfinite(time) || time != left * 1e-9 || left != right)
      throw std::runtime_error("Invalid synchronous camera receipt");
    if (id && left <= camera_ns)
      throw std::runtime_error("Camera receipt integer timestamp is not strictly increasing");
    ++id;
    camera_ns = left;
    right_camera_ns = right;
    physical_camera_time = time;
    pending_ = true;
  }
  void validate(int64_t original_ns, double frame_time, bool available, uint64_t epoch, uint64_t sequence) const {
    if (!pending_ || original_ns != camera_ns)
      throw std::runtime_error("Missing or duplicate camera receipt");
    if (available) {
      if (frame_time != physical_camera_time)
        throw std::runtime_error("Available observer frame belongs to another camera receipt");
      if (have_available_ && (epoch < last_epoch_ || (epoch == last_epoch_ && sequence <= last_sequence_)))
        throw std::runtime_error("Observer event identity repeated or regressed");
    }
  }
  static void require_written(std::ostream &out) {
    if (!out)
      throw std::runtime_error("Failed to write camera receipt/feature diagnostics");
  }
  void complete(int64_t original_ns, double frame_time, bool available, uint64_t epoch, uint64_t sequence) {
    validate(original_ns, frame_time, available, epoch, sequence);
    if (available) {
      last_epoch_ = epoch;
      last_sequence_ = sequence;
      have_available_ = true;
    }
    pending_ = false;
  }

private:
  bool pending_ = false, have_available_ = false;
  uint64_t last_epoch_ = 0, last_sequence_ = 0;
};
