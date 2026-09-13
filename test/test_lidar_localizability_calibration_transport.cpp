#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "lidar_localizability_calibration_transport.h"

namespace
{

using CalibrationMessage =
    fast_livo::msg::LidarLocalizabilityCalibration;
using lidar_localizability_calibration_transport::
    serializeLidarLocalizabilityCalibration;
using CalibrationTransportPolicy =
    lidar_localizability_calibration_transport::Policy;

constexpr double kTolerance = 1e-12;
constexpr std::uint64_t kInputSamples =
    (std::uint64_t{1} << 40U) + 29U;
constexpr std::uint64_t kUsedSamples =
    (std::uint64_t{1} << 39U) + 23U;
constexpr std::uint64_t kHistogramCountBase =
    std::uint64_t{1} << 40U;

template<typename Message, typename = void>
struct HasMeasurementInformation : std::false_type
{
};

template<typename Message>
struct HasMeasurementInformation<
    Message,
    std::void_t<decltype(
        std::declval<Message>().measurement_information)>>
    : std::true_type
{
};

template<typename Message, typename = void>
struct HasClassification : std::false_type
{
};

template<typename Message>
struct HasClassification<
    Message,
    std::void_t<decltype(
        std::declval<Message>().classification)>>
    : std::true_type
{
};

Eigen::Matrix3d
deserializeMatrixRowMajor(
    const std::array<double, 9> &serialized)
{
  Eigen::Matrix3d matrix;

  for (Eigen::Index row = 0; row < 3; ++row)
  {
    for (Eigen::Index column = 0; column < 3; ++column)
    {
      matrix(row, column) =
          serialized[static_cast<std::size_t>(3 * row + column)];
    }
  }

  return matrix;
}

LidarLocalizabilityCalibrationSummary
makeAvailableSummary()
{
  LidarLocalizabilityCalibrationSummary summary;

  summary.available = true;
  summary.basis.available = true;
  summary.basis.input_samples =
      static_cast<std::size_t>(kInputSamples);
  summary.basis.used_samples =
      static_cast<std::size_t>(kUsedSamples);
  summary.histogram_bins = 2U;

  const Eigen::AngleAxisd rotation(
      0.73,
      Eigen::Vector3d(1.0, -2.0, 3.0).normalized());
  summary.basis.rotation_eigenvectors =
      rotation.toRotationMatrix();
  summary.basis.rotation_eigenvalues <<
      2.0, 5.0, 11.0;
  summary.basis.rotation_gram_matrix =
      summary.basis.rotation_eigenvectors *
      summary.basis.rotation_eigenvalues.asDiagonal() *
      summary.basis.rotation_eigenvectors.transpose();

  const Eigen::AngleAxisd translation(
      -0.41,
      Eigen::Vector3d(-2.0, 1.0, 4.0).normalized());
  summary.basis.translation_eigenvectors =
      translation.toRotationMatrix();
  summary.basis.translation_eigenvalues <<
      0.25, 1.5, 8.0;
  summary.basis.translation_gram_matrix =
      summary.basis.translation_eigenvectors *
      summary.basis.translation_eigenvalues.asDiagonal() *
      summary.basis.translation_eigenvectors.transpose();

  for (std::size_t direction = 0; direction < 3U; ++direction)
  {
    const std::uint64_t count_base =
        kHistogramCountBase +
        static_cast<std::uint64_t>(100U * direction);
    const double sum_base =
        10.0 * static_cast<double>(direction);

    summary.rotation[direction].count =
        {count_base + 1U, count_base + 2U};
    summary.rotation[direction].sum =
        {sum_base + 0.1, sum_base + 0.2};
    summary.translation[direction].count =
        {count_base + 3U, count_base + 4U};
    summary.translation[direction].sum =
        {sum_base + 0.3, sum_base + 0.4};
  }

  return summary;
}

}  // namespace

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    DefaultIsInactiveAndHasNoInventedDepth)
{
  const LidarLocalizabilityCalibrationPolicy calibration_policy;
  const CalibrationTransportPolicy transport_policy;

  EXPECT_FALSE(transport_policy.active(calibration_policy));
  EXPECT_FALSE(transport_policy.publish_enabled);
  EXPECT_EQ(transport_policy.topic, "/lidar_localizability_calibration");
  EXPECT_EQ(transport_policy.qos_depth, 0);
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    CalibrationInactiveDisablesOtherwiseValidTransport)
{
  LidarLocalizabilityCalibrationPolicy calibration_policy;
  calibration_policy.enabled = false;
  calibration_policy.histogram_bins = 8;

  CalibrationTransportPolicy transport_policy;
  transport_policy.publish_enabled = true;
  transport_policy.qos_depth = 3;

  EXPECT_FALSE(transport_policy.active(calibration_policy));
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    PublishDisabledIsInactive)
{
  LidarLocalizabilityCalibrationPolicy calibration_policy;
  calibration_policy.enabled = true;
  calibration_policy.histogram_bins = 8;

  CalibrationTransportPolicy transport_policy;
  transport_policy.qos_depth = 3;

  EXPECT_FALSE(transport_policy.active(calibration_policy));
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    NonPositiveDepthIsInactive)
{
  LidarLocalizabilityCalibrationPolicy calibration_policy;
  calibration_policy.enabled = true;
  calibration_policy.histogram_bins = 8;

  CalibrationTransportPolicy transport_policy;
  transport_policy.publish_enabled = true;

  transport_policy.qos_depth = 0;
  EXPECT_FALSE(transport_policy.active(calibration_policy));

  transport_policy.qos_depth = -1;
  EXPECT_FALSE(transport_policy.active(calibration_policy));
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    EmptyTopicIsInactive)
{
  LidarLocalizabilityCalibrationPolicy calibration_policy;
  calibration_policy.enabled = true;
  calibration_policy.histogram_bins = 8;

  CalibrationTransportPolicy transport_policy;
  transport_policy.publish_enabled = true;
  transport_policy.qos_depth = 3;
  transport_policy.topic.clear();

  EXPECT_FALSE(transport_policy.active(calibration_policy));
  EXPECT_TRUE(transport_policy.topic.empty());
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    AllConditionsValidActivatesTransport)
{
  LidarLocalizabilityCalibrationPolicy calibration_policy;
  calibration_policy.enabled = true;
  calibration_policy.histogram_bins = 8;

  CalibrationTransportPolicy transport_policy;
  transport_policy.publish_enabled = true;
  transport_policy.qos_depth = 3;

  EXPECT_TRUE(transport_policy.active(calibration_policy));
}

TEST(
    LidarLocalizabilityCalibrationTransportPolicy,
    QosIsReliableVolatileKeepLastWithRequestedDepth)
{
  constexpr std::size_t kRequestedDepth = 7U;
  const auto qos =
      lidar_localizability_calibration_transport::makeQos(
          kRequestedDepth);

  EXPECT_EQ(qos.history(), rclcpp::HistoryPolicy::KeepLast);
  EXPECT_EQ(qos.depth(), kRequestedDepth);
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST(
    LidarLocalizabilityCalibrationTransport,
    SerializesAsymmetricMatricesRowMajor)
{
  auto summary = makeAvailableSummary();
  summary.basis.rotation_gram_matrix <<
      1.0, 2.0, 3.0,
      4.0, 5.0, 6.0,
      7.0, 8.0, 9.0;
  summary.basis.translation_gram_matrix <<
      -1.0, -2.0, -3.0,
      -4.0, -5.0, -6.0,
      -7.0, -8.0, -9.0;

  const auto message =
      serializeLidarLocalizabilityCalibration(summary);

  EXPECT_EQ(
      message.rotation_gram_matrix,
      (std::array<double, 9>{
          1.0, 2.0, 3.0,
          4.0, 5.0, 6.0,
          7.0, 8.0, 9.0}));
  EXPECT_EQ(
      message.translation_gram_matrix,
      (std::array<double, 9>{
          -1.0, -2.0, -3.0,
          -4.0, -5.0, -6.0,
          -7.0, -8.0, -9.0}));
}

TEST(
    LidarLocalizabilityCalibrationTransport,
    PreservesEigenvectorColumnsAndRawCoefficients)
{
  const auto summary = makeAvailableSummary();
  const auto message =
      serializeLidarLocalizabilityCalibration(summary);

  const Eigen::Matrix3d rotation_gram =
      deserializeMatrixRowMajor(message.rotation_gram_matrix);
  const Eigen::Matrix3d rotation_eigenvectors =
      deserializeMatrixRowMajor(message.rotation_eigenvectors);
  const Eigen::Matrix3d translation_gram =
      deserializeMatrixRowMajor(message.translation_gram_matrix);
  const Eigen::Matrix3d translation_eigenvectors =
      deserializeMatrixRowMajor(message.translation_eigenvectors);

  EXPECT_EQ(
      message.rotation_eigenvalues,
      (std::array<double, 3>{2.0, 5.0, 11.0}));
  EXPECT_EQ(
      message.translation_eigenvalues,
      (std::array<double, 3>{0.25, 1.5, 8.0}));

  for (Eigen::Index index = 0; index < 3; ++index)
  {
    EXPECT_TRUE(
        (rotation_gram * rotation_eigenvectors.col(index))
            .isApprox(
                message.rotation_eigenvalues[
                    static_cast<std::size_t>(index)] *
                rotation_eigenvectors.col(index),
                kTolerance));
    EXPECT_TRUE(
        (translation_gram * translation_eigenvectors.col(index))
            .isApprox(
                message.translation_eigenvalues[
                    static_cast<std::size_t>(index)] *
                translation_eigenvectors.col(index),
                kTolerance));
  }

  for (Eigen::Index row = 0; row < 3; ++row)
  {
    for (Eigen::Index column = 0; column < 3; ++column)
    {
      const auto serialized_index =
          static_cast<std::size_t>(3 * row + column);

      EXPECT_DOUBLE_EQ(
          message.rotation_eigenvectors[serialized_index],
          summary.basis.rotation_eigenvectors(row, column));
      EXPECT_DOUBLE_EQ(
          message.translation_eigenvectors[serialized_index],
          summary.basis.translation_eigenvectors(row, column));
    }
  }
}

TEST(
    LidarLocalizabilityCalibrationTransport,
    PreservesCountsRadiusAndDirectionMajorHistograms)
{
  const auto summary = makeAvailableSummary();
  const auto message =
      serializeLidarLocalizabilityCalibration(summary);

  EXPECT_TRUE(message.available);
  EXPECT_EQ(message.input_samples, kInputSamples);
  EXPECT_EQ(message.used_samples, kUsedSamples);
  EXPECT_EQ(message.histogram_bins, 2U);
  EXPECT_DOUBLE_EQ(
      message.rotation_contribution_normalization_radius_m,
      kLidarMomentNormalizationRadiusM);

  ASSERT_EQ(message.rotation_histogram_count.size(), 6U);
  ASSERT_EQ(message.rotation_histogram_sum.size(), 6U);
  ASSERT_EQ(message.translation_histogram_count.size(), 6U);
  ASSERT_EQ(message.translation_histogram_sum.size(), 6U);

  EXPECT_EQ(
      message.rotation_histogram_count,
      (std::vector<std::uint64_t>{
          kHistogramCountBase + 1U,
          kHistogramCountBase + 2U,
          kHistogramCountBase + 101U,
          kHistogramCountBase + 102U,
          kHistogramCountBase + 201U,
          kHistogramCountBase + 202U}));
  EXPECT_EQ(
      message.rotation_histogram_sum,
      (std::vector<double>{
          0.1, 0.2, 10.1, 10.2, 20.1, 20.2}));
  EXPECT_EQ(
      message.translation_histogram_count,
      (std::vector<std::uint64_t>{
          kHistogramCountBase + 3U,
          kHistogramCountBase + 4U,
          kHistogramCountBase + 103U,
          kHistogramCountBase + 104U,
          kHistogramCountBase + 203U,
          kHistogramCountBase + 204U}));
  EXPECT_EQ(
      message.translation_histogram_sum,
      (std::vector<double>{
          0.3, 0.4, 10.3, 10.4, 20.3, 20.4}));
}

TEST(
    LidarLocalizabilityCalibrationTransport,
    UnavailableSummaryRemainsUnavailableWithoutSynthesizedHistograms)
{
  LidarLocalizabilityCalibrationSummary summary;
  summary.basis.input_samples = 17U;
  summary.basis.used_samples = 13U;
  summary.histogram_bins = 4U;

  const auto message =
      serializeLidarLocalizabilityCalibration(summary);

  EXPECT_FALSE(message.available);
  EXPECT_EQ(message.input_samples, 17U);
  EXPECT_EQ(message.used_samples, 13U);
  EXPECT_EQ(message.histogram_bins, 4U);
  EXPECT_TRUE(message.rotation_histogram_count.empty());
  EXPECT_TRUE(message.rotation_histogram_sum.empty());
  EXPECT_TRUE(message.translation_histogram_count.empty());
  EXPECT_TRUE(message.translation_histogram_sum.empty());
}

TEST(
    LidarLocalizabilityCalibrationTransport,
    ExcludesInformationAndClassificationFields)
{
  static_assert(!HasMeasurementInformation<CalibrationMessage>::value);
  static_assert(!HasClassification<CalibrationMessage>::value);

  SUCCEED();
}
