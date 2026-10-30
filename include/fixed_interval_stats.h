#ifndef FIXED_INTERVAL_STATS_H
#define FIXED_INTERVAL_STATS_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

template<std::size_t Capacity>
class FixedIntervalStats
{
public:
  static_assert(Capacity > 0, "FixedIntervalStats capacity must be nonzero");

  // Percentiles describe only the bounded recent window. max_all covers every
  // sample added since reset, including samples evicted from that window.
  struct Summary
  {
    std::uint64_t sample_count;
    std::size_t recent_sample_count;
    double p50_recent;
    double p95_recent;
    double max_all;
  };

  void add(double value)
  {
    values_[next_] = value;
    next_ = (next_ + 1) % Capacity;
    if (stored_ < Capacity) ++stored_;
    if (count_ == 0 || value > max_all_) max_all_ = value;
    ++count_;
  }

  Summary summary() const
  {
    if (stored_ == 0) return {0, 0, NAN, NAN, NAN};
    auto sorted = values_;
    std::sort(sorted.begin(), sorted.begin() + stored_);
    const auto percentile = [&](double fraction) {
      return sorted[static_cast<std::size_t>(std::ceil(fraction * stored_)) - 1];
    };
    return {count_, stored_, percentile(0.50), percentile(0.95), max_all_};
  }

  void reset()
  {
    next_ = 0;
    stored_ = 0;
    count_ = 0;
    max_all_ = std::numeric_limits<double>::quiet_NaN();
  }

private:
  std::array<double, Capacity> values_{};
  std::size_t next_ = 0;
  std::size_t stored_ = 0;
  std::uint64_t count_ = 0;
  double max_all_ = std::numeric_limits<double>::quiet_NaN();
};

#endif
