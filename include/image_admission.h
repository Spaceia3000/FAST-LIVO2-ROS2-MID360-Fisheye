#ifndef IMAGE_ADMISSION_H
#define IMAGE_ADMISSION_H

#include <cmath>

enum class ImageAdmissionDecision
{
  ACCEPT,
  DUPLICATE,
  OUT_OF_ORDER,
  TOO_SOON,
};

inline ImageAdmissionDecision imageAdmissionDecision(
    const double corrected_timestamp,
    const double last_accepted_timestamp,
    const double minimum_interval)
{
  if (std::abs(corrected_timestamp - last_accepted_timestamp) < 0.001)
  {
    return ImageAdmissionDecision::DUPLICATE;
  }
  if (corrected_timestamp < last_accepted_timestamp)
  {
    return ImageAdmissionDecision::OUT_OF_ORDER;
  }
  if (corrected_timestamp - last_accepted_timestamp < minimum_interval)
  {
    return ImageAdmissionDecision::TOO_SOON;
  }
  return ImageAdmissionDecision::ACCEPT;
}

#endif
