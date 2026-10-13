#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <new>

#include "IMU_Processing.h"

namespace
{

class PatternInitializedImuProcess
{
public:
  PatternInitializedImuProcess()
  {
    std::memset(storage_.data(), 0x43, storage_.size());
    process_ = new (storage_.data()) ImuProcess();
  }

  ~PatternInitializedImuProcess()
  {
    process_->~ImuProcess();
  }

  ImuProcess & get()
  {
    return *process_;
  }

private:
  alignas(ImuProcess) std::array<unsigned char, sizeof(ImuProcess)> storage_;
  ImuProcess * process_;
};

sensor_msgs::msg::Imu::SharedPtr makeImu(
  const double stamp,
  const double angular_velocity_z)
{
  auto imu = std::make_shared<sensor_msgs::msg::Imu>();
  const double seconds = std::floor(stamp);
  imu->header.stamp.sec = static_cast<int32_t>(seconds);
  imu->header.stamp.nanosec = static_cast<uint32_t>(
    std::llround((stamp - seconds) * 1e9));
  imu->linear_acceleration.z = -G_m_s2;
  imu->angular_velocity.z = angular_velocity_z;
  return imu;
}

void initializeAt(
  ImuProcess & process,
  LidarMeasureGroup & lidar_measures,
  StatesGroup & state,
  const double stamp)
{
  process.set_imu_init_frame_num(1);

  MeasureGroup initialization;
  initialization.lio_time = stamp;
  initialization.imu.push_back(makeImu(stamp, 1.0));

  lidar_measures.lio_vio_flg = LIO;
  lidar_measures.measures.clear();
  lidar_measures.measures.push_back(initialization);

  auto output = std::make_shared<PointCloudXYZI>();
  process.Process2(lidar_measures, state, output);
  ASSERT_FALSE(process.imu_need_init);
}

void propagateTo(
  ImuProcess & process,
  LidarMeasureGroup & lidar_measures,
  StatesGroup & state,
  const double stamp)
{
  MeasureGroup propagation;
  propagation.lio_time = stamp;
  propagation.imu.push_back(makeImu(stamp, 1.0));

  PointType point;
  point.x = 1.0F;
  point.curvature = 50.0F;
  lidar_measures.pcl_proc_cur->clear();
  lidar_measures.pcl_proc_cur->push_back(point);
  lidar_measures.lio_vio_flg = LIO;
  lidar_measures.measures.clear();
  lidar_measures.measures.push_back(propagation);

  auto output = std::make_shared<PointCloudXYZI>();
  process.Process2(lidar_measures, state, output);
}

double yaw(const StatesGroup & state)
{
  return std::atan2(state.rot_end(1, 0), state.rot_end(0, 0));
}

TEST(ImuInitializationTimeOrigin, UsesLastInitializationMeasurementAtNonzeroEpoch)
{
  PatternInitializedImuProcess storage;
  auto & process = storage.get();
  LidarMeasureGroup lidar_measures;
  StatesGroup state;

  initializeAt(process, lidar_measures, state, 1000.0);
  propagateTo(process, lidar_measures, state, 1000.1);

  EXPECT_NEAR(yaw(state), 0.1, 1e-6);
  EXPECT_TRUE(state.pos_end.allFinite());
  EXPECT_TRUE(state.vel_end.allFinite());
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_NEAR(lidar_measures.last_lio_update_time, 1000.1, 1e-9);
}

TEST(ImuInitializationTimeOrigin, ResetRebasesAcrossClockRestart)
{
  PatternInitializedImuProcess storage;
  auto & process = storage.get();
  LidarMeasureGroup lidar_measures;
  StatesGroup state;

  initializeAt(process, lidar_measures, state, 1000.0);
  propagateTo(process, lidar_measures, state, 1000.1);
  // This call stores the legacy endpoint even when an invalid origin skips its
  // state update. The assertion below therefore isolates Reset/reinit behavior.

  process.Reset();
  state = StatesGroup();
  initializeAt(process, lidar_measures, state, 100.0);
  propagateTo(process, lidar_measures, state, 100.05);

  EXPECT_NEAR(yaw(state), 0.05, 1e-6);
  EXPECT_TRUE(state.pos_end.allFinite());
  EXPECT_TRUE(state.vel_end.allFinite());
  EXPECT_TRUE(state.cov.allFinite());
  EXPECT_NEAR(lidar_measures.last_lio_update_time, 100.05, 1e-9);
}

}  // namespace
