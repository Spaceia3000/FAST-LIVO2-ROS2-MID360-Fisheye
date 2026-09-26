
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
namespace rclcpp {
inline bool ok() { return true; }
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher<T>>;
  T captured;
  void publish(const T &value) { captured = value; }
};
}
struct Broadcaster {
  geometry_msgs::msg::TransformStamped captured;
  void sendTransform(const geometry_msgs::msg::TransformStamped &value) { captured = value; }
};
struct State {
  double values[3];
  double pos_end(int i) const { return values[i]; }
};
struct LIVMapper {
  State _state;
  struct { double last_lio_update_time = 1234.25; } LidarMeasures;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  std::shared_ptr<Broadcaster> tf_broadcaster = std::make_shared<Broadcaster>();
  template<class T> void set_posestamp(T &out);
  void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &);
};
builtin_interfaces::msg::Time sec2Stamp(double value) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int>(value);
  stamp.nanosec = static_cast<unsigned>((value - stamp.sec) * 1e9);
  return stamp;
}
// PUBLICATION_FUNCTIONS

bool same(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
int main() {
  LIVMapper mapper;
  auto publisher = std::make_shared<rclcpp::Publisher<nav_msgs::msg::Odometry>>();
  for (int i = 0; i < 10000; ++i) {
    const double angle = 0.00031 * i;
    mapper._state = {{std::nextafter(angle, 10.0), -0.0, -123.456789}};
    mapper.geoQuat.x = std::sin(angle) / std::sqrt(14.0);
    mapper.geoQuat.y = 2 * std::sin(angle) / std::sqrt(14.0);
    mapper.geoQuat.z = 3 * std::sin(angle) / std::sqrt(14.0);
    mapper.geoQuat.w = std::cos(angle);
    mapper.publish_odometry(publisher);
    const auto &odom = publisher->captured;
    const auto &tf = mapper.tf_broadcaster->captured;
    assert(odom.header == tf.header);
    assert(odom.header.stamp == sec2Stamp(mapper.LidarMeasures.last_lio_update_time));
    assert(odom.header.frame_id == "camera_init");
    assert(odom.child_frame_id == "aft_mapped" && odom.child_frame_id == tf.child_frame_id);
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    const auto &t = tf.transform.translation;
    const auto &r = tf.transform.rotation;
    assert(same(p.x, t.x) && same(p.y, t.y) && same(p.z, t.z));
    assert(same(q.x, r.x) && same(q.y, r.y) && same(q.z, r.z) && same(q.w, r.w));
    assert(same(q.x, mapper.geoQuat.x) && same(q.w, mapper.geoQuat.w));
  }
}
