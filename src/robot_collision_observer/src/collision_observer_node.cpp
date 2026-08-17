#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gazebo_msgs/msg/contact_state.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using namespace std::chrono_literals;

constexpr double kFreshnessTimeout = 0.25;
constexpr double kReleaseGrace = 0.05;

const std::map<std::string, std::string> kContactTopics = {
  {"base", "/gazebo/contacts/base"},
  {"lidar", "/gazebo/contacts/lidar"},
  {"wheel_left", "/gazebo/contacts/wheel_left"},
  {"wheel_right", "/gazebo/contacts/wheel_right"},
  {"caster", "/gazebo/contacts/caster"},
};

const std::map<std::string, std::string> kRobotCollisions = {
  {"base", "turtlebot3_burger_low_lidar::base_link::base_collision"},
  {"lidar", "turtlebot3_burger_low_lidar::base_scan::lidar_sensor_collision"},
  {"wheel_left",
    "turtlebot3_burger_low_lidar::wheel_left_link::wheel_left_collision"},
  {"wheel_right",
    "turtlebot3_burger_low_lidar::wheel_right_link::wheel_right_collision"},
  {"caster",
    "turtlebot3_burger_low_lidar::caster_back_link::caster_collision"},
};

const std::vector<std::string> kApprovedFloors = {
  "competition_field::field_link::floor_collision",
  "ground_plane::link::collision",
};

bool ends_with_scoped(const std::string & actual, const std::string & expected)
{
  if (actual == expected) {
    return true;
  }
  const std::string suffix = "::" + expected;
  return actual.size() >= suffix.size() &&
         actual.compare(actual.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_ground_sensor(const std::string & sensor)
{
  return sensor == "wheel_left" || sensor == "wheel_right" || sensor == "caster";
}

bool is_approved_floor(const std::string & value)
{
  return std::any_of(
    kApprovedFloors.begin(), kApprovedFloors.end(),
    [&value](const auto & floor) {return ends_with_scoped(value, floor);});
}

bool is_approved_ground_contact(
  const std::string & sensor,
  const std::string & first,
  const std::string & second)
{
  if (!is_ground_sensor(sensor)) {
    return false;
  }
  const auto & robot = kRobotCollisions.at(sensor);
  return (ends_with_scoped(first, robot) && is_approved_floor(second)) ||
         (ends_with_scoped(second, robot) && is_approved_floor(first));
}

std::string pair_key(const std::string & first, const std::string & second)
{
  return first < second ? first + "\x1f" + second : second + "\x1f" + first;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class CollisionObserver : public rclcpp::Node
{
public:
  CollisionObserver()
  : Node("robot_collision_observer")
  {
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/acceptance/collision_status", 10);
    event_publisher_ = create_publisher<std_msgs::msg::String>(
      "/acceptance/collision_events", 10);
    for (const auto & [sensor, topic] : kContactTopics) {
      subscriptions_.push_back(create_subscription<gazebo_msgs::msg::ContactsState>(
        topic, rclcpp::SensorDataQoS(),
        [this, sensor](gazebo_msgs::msg::ContactsState::SharedPtr message) {
          contact_callback(sensor, *message);
        }));
    }
    status_timer_ = create_wall_timer(100ms, [this]() {publish_status();});
  }

private:
  void contact_callback(
    const std::string & sensor,
    const gazebo_msgs::msg::ContactsState & message)
  {
    const double now = steady_seconds();
    last_received_[sensor] = now;
    for (const auto & state : message.states) {
      if (state.collision1_name.empty() || state.collision2_name.empty() ||
        is_approved_ground_contact(
          sensor, state.collision1_name, state.collision2_name))
      {
        continue;
      }
      const auto key = pair_key(state.collision1_name, state.collision2_name);
      const auto active = active_until_.find(key);
      if (active == active_until_.end() || active->second < now) {
        ++collision_count_;
        publish_event(sensor, state, now);
        RCLCPP_ERROR(
          get_logger(), "Unexpected contact: %s <-> %s",
          state.collision1_name.c_str(), state.collision2_name.c_str());
      }
      active_until_[key] = now + kReleaseGrace;
    }
    for (auto iterator = active_until_.begin(); iterator != active_until_.end();) {
      if (iterator->second < now) {
        iterator = active_until_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void publish_event(
    const std::string & sensor,
    const gazebo_msgs::msg::ContactState & state,
    double observed_at)
  {
    std::ostringstream json;
    json << std::setprecision(15)
         << "{\"event_type\":\"collision\","
         << "\"sensor\":\"" << json_escape(sensor) << "\","
         << "\"collision_count\":" << collision_count_ << ','
         << "\"collision1_name\":\""
         << json_escape(state.collision1_name) << "\","
         << "\"collision2_name\":\""
         << json_escape(state.collision2_name) << "\","
         << "\"observed_at_monotonic_s\":" << observed_at << ','
         << "\"contact_positions\":[";
    for (std::size_t index = 0; index < state.contact_positions.size(); ++index) {
      if (index > 0) {json << ',';}
      const auto & point = state.contact_positions[index];
      json << "{\"x\":" << point.x << ",\"y\":" << point.y
           << ",\"z\":" << point.z << '}';
    }
    json << "],\"depths\":[";
    for (std::size_t index = 0; index < state.depths.size(); ++index) {
      if (index > 0) {json << ',';}
      json << state.depths[index];
    }
    json << "]}";
    std_msgs::msg::String output;
    output.data = json.str();
    event_publisher_->publish(output);
  }

  void publish_status()
  {
    const double now = steady_seconds();
    std::vector<std::string> stale;
    for (const auto & [sensor, topic] : kContactTopics) {
      (void)topic;
      const auto sample = last_received_.find(sensor);
      if (sample == last_received_.end() || now - sample->second > kFreshnessTimeout) {
        stale.push_back(sensor);
      }
    }
    std::ostringstream json;
    json << std::setprecision(15)
         << "{\"valid\":" << (stale.empty() ? "true" : "false") << ','
         << "\"collision_count\":" << collision_count_ << ','
         << "\"all_sensors_seen\":"
         << (last_received_.size() == kContactTopics.size() ? "true" : "false")
         << ",\"all_sensors_fresh\":"
         << (stale.empty() ? "true" : "false")
         << ",\"stale_sensors\":[";
    for (std::size_t index = 0; index < stale.size(); ++index) {
      if (index > 0) {json << ',';}
      json << '\"' << json_escape(stale[index]) << '\"';
    }
    json << "],\"sensor_ages_s\":{";
    std::size_t index = 0;
    for (const auto & [sensor, topic] : kContactTopics) {
      (void)topic;
      if (index++ > 0) {json << ',';}
      json << '\"' << json_escape(sensor) << "\":";
      const auto sample = last_received_.find(sensor);
      if (sample == last_received_.end()) {
        json << "null";
      } else {
        json << std::max(0.0, now - sample->second);
      }
    }
    json << "}}";
    std_msgs::msg::String output;
    output.data = json.str();
    status_publisher_->publish(output);
  }

  std::vector<rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr>
    subscriptions_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unordered_map<std::string, double> last_received_;
  std::unordered_map<std::string, double> active_until_;
  std::size_t collision_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionObserver>());
  rclcpp::shutdown();
  return 0;
}
