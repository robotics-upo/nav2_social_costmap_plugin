#include "nav2_social_costmap_plugin/social_layer.hpp"

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <vector>

// START INCLUDES from socialLayer
#include "std_msgs/msg/string.hpp"
#include <angles/angles.h>
#include <math.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// END INCLUDES from socialLayer

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_social_costmap_plugin
{

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and subscription to topics
void SocialLayer::onInitialize()
{

  // START Subscription to topic
  auto nod = node_.lock();
  declareParameter("people_topic", rclcpp::ParameterValue("/people"));
  nod->get_parameter(name_ + "." + "people_topic", people_topic_);

  ppl_sub_ = nod->create_subscription<people_msgs::msg::People>(
    people_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    nod->get_logger(),
    "SocialLayer: subscribed to "
    "topic %s",
    ppl_sub_->get_topic_name());

  // Whether to apply this plugin or not
  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Smallest value to publish on costmap adjustments - not modify
  declareParameter("cutoff", rclcpp::ParameterValue(5.0));
  // Amplitude of adjustments at peak [0,254] or [0,100], we can keep it fixed
  declareParameter("amplitude", rclcpp::ParameterValue(255.0));
  // Covariance of adjustments [0, 1]
  declareParameter("covariance_front_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_front_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_rear_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_rear_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_right_height", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_right_width", rclcpp::ParameterValue(0.25));
  declareParameter("covariance_when_still", rclcpp::ParameterValue(0.25));
  declareParameter("use_passing", rclcpp::ParameterValue(true));
  declareParameter("use_vel_factor", rclcpp::ParameterValue(true));
  // Factor with which to scale the velocity [1-10]
  declareParameter("speed_factor_multiplier", rclcpp::ParameterValue(5.0));
  declareParameter("publish_occgrid", rclcpp::ParameterValue(false));
  get_parameters();

  tolerance_vel_still_ = 0.1;

  if (publish_occgrid_) {
    grid_pub_ =
      nod->create_publisher<nav_msgs::msg::OccupancyGrid>("social_grid", 1);
    grid_pub_->on_activate();
  }
}

void SocialLayer::get_parameters()
{
  auto nod = node_.lock();
  nod->get_parameter(name_ + "." + "enabled", enabled_);
  nod->get_parameter(name_ + "." + "cutoff", cutoff_);
  nod->get_parameter(name_ + "." + "amplitude", amplitude_);
  nod->get_parameter(
    name_ + "." + "covariance_front_height",
    sigma_front_height_);
  nod->get_parameter(
    name_ + "." + "covariance_front_width",
    sigma_front_width_);
  nod->get_parameter(
    name_ + "." + "covariance_rear_height",
    sigma_rear_height_);
  nod->get_parameter(
    name_ + "." + "covariance_rear_width",
    sigma_rear_width_);
  nod->get_parameter(
    name_ + "." + "covariance_right_height",
    sigma_right_height_);
  nod->get_parameter(
    name_ + "." + "covariance_right_width",
    sigma_right_width_);
  nod->get_parameter(
    name_ + "." + "covariance_when_still",
    sigma_when_still_);
  nod->get_parameter(name_ + "." + "use_passing", use_passing_);
  nod->get_parameter(name_ + "." + "use_vel_factor", use_vel_factor_);
  nod->get_parameter(name_ + "." + "speed_factor_multiplier", speed_factor_);
  nod->get_parameter(name_ + "." + "publish_occgrid", publish_occgrid_);
}

void SocialLayer::peopleCallback(
  const people_msgs::msg::People::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(ppl_message_mutex_);
  people_list_ = msg;
}

void SocialLayer::onFootprintChanged()
{
  return;
}

void SocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  std::shared_ptr<const people_msgs::msg::People> people_copy;
  {
    std::lock_guard<std::mutex> lock(ppl_message_mutex_);
    people_copy = people_list_;
  }

  if (!people_copy || people_copy->people.empty()) {
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    return;
  }

  std::string global_frame =
    layered_costmap_
    ->getGlobalFrameID();         // Returns the global frame of the costmap
  std::vector<people_msgs::msg::Person> transformed_people_local;
  transformed_people_local.reserve(people_copy->people.size());
  int cnt_j = 0;

  for (const auto & person_input : people_copy->people) {
    people_msgs::msg::Person tpt;
    geometry_msgs::msg::PointStamped pt, opt;

    pt.point.x = person_input.position.x;
    pt.point.y = person_input.position.y;
    pt.point.z = person_input.position.z;
    pt.header.frame_id = people_copy->header.frame_id;
    pt.header.stamp = people_copy->header.stamp;

  if (!tf_->canTransform(
    pt.header.frame_id, global_frame,
    tf2_ros::fromMsg(pt.header.stamp)))
    {
      RCLCPP_INFO(
        logger_,
        "Social layer can't transform from %s to %s",
        pt.header.frame_id.c_str(), global_frame.c_str());
      return;
    }

    tf_->transform(pt, opt, global_frame);
    tpt.position.x = opt.point.x;
    tpt.position.y = opt.point.y;
    tpt.position.z = opt.point.z;

    pt.point.x += person_input.velocity.x;
    pt.point.y += person_input.velocity.y;
    pt.point.z += person_input.velocity.z;
    tf_->transform(pt, opt, global_frame);

    tpt.velocity.x = opt.point.x - tpt.position.x;
    tpt.velocity.y = opt.point.y - tpt.position.y;
    tpt.velocity.z = opt.point.z - tpt.position.z;

    cnt_j++;
    transformed_people_local.push_back(tpt);
  }

  {
    std::lock_guard<std::mutex> lock(transformed_people_mutex_);
    transformed_people_ = transformed_people_local;
  }

  for (const auto & person : transformed_people_local) {

    double mag = sqrt(pow(person.velocity.x, 2) + pow(person.velocity.y, 2));
    double greater = get_radius(cutoff_, amplitude_, sigma_when_still_);
    if (mag >= tolerance_vel_still_) {
      double front_height =
        get_radius(cutoff_, amplitude_, sigma_front_height_);
      if (use_vel_factor_) {
        double factor = 1.0 + mag * speed_factor_;
        front_height =
          get_radius(cutoff_, amplitude_, sigma_front_height_ * factor);
      }
      double rear_height = get_radius(cutoff_, amplitude_, sigma_rear_height_);

      double front_width = get_radius(cutoff_, amplitude_, sigma_front_width_);
      double rear_width = get_radius(cutoff_, amplitude_, sigma_rear_width_);
      double right_height = 0.0;
      if (use_passing_) {
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);
      }

      greater = std::max(
        front_height,
        std::max(
          rear_height,
          std::max(right_height, std::max(front_width, rear_width))));
    }

    *min_x = std::min(*min_x, person.position.x - greater);
    *min_y = std::min(*min_y, person.position.y - greater);
    *max_x = std::max(*max_x, person.position.x + greater);
    *max_y = std::max(*max_y, person.position.y + greater);
  }
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{

  if (!enabled_) {
    RCLCPP_INFO(logger_, "SocialLayer::updateCosts. Disabled");
    current_ = true;
    return;
  }

  std::shared_ptr<const people_msgs::msg::People> people_snapshot;
  {
    std::lock_guard<std::mutex> lock(ppl_message_mutex_);
    people_snapshot = people_list_;
  }
  if (!people_snapshot || people_snapshot->people.empty()) {
    RCLCPP_INFO(logger_, "SocialLayer::updateCosts. No people");
    current_ = true;
    return;
  }

  if (cutoff_ >= amplitude_) {
    RCLCPP_INFO(
      logger_, "SocialLayer::updateCosts. cutoff: %f, amplitude: %f", cutoff_,
      amplitude_);
    current_ = true;
    return;
  }

  get_parameters();

  double res = master_grid.getResolution();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = clock_->now();
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();

  grid.info.height = master_grid.getSizeInCellsY();
  grid.info.width = master_grid.getSizeInCellsX();
  grid.info.resolution = res;
  grid.info.origin.position.x = master_grid.getOriginX();
  grid.info.origin.position.y = master_grid.getOriginY();
  grid.info.origin.orientation.w = 1.0;
  std::vector<int8_t> vect((grid.info.height * grid.info.width), 0);
  grid.data = vect;

  std::vector<people_msgs::msg::Person> transformed_people_snapshot;
  {
    std::lock_guard<std::mutex> lock(transformed_people_mutex_);
    transformed_people_snapshot = transformed_people_;
  }

  for (const auto & person : transformed_people_snapshot) {
    double mag = sqrt(
      person.velocity.x * person.velocity.x +
      person.velocity.y * person.velocity.y);
    double angle = atan2(person.velocity.y, person.velocity.x);
    double angle_right = angle - 1.57;   // 1.51;
    double radius = get_radius(cutoff_, amplitude_, sigma_when_still_);
    double front_height = radius;
    double rear_height = radius;
    double greater_side = radius + radius;
    if (mag >= tolerance_vel_still_) {
      if (use_vel_factor_) {
        double factor = 1.0 + mag * speed_factor_;
        front_height =
          get_radius(cutoff_, amplitude_, sigma_front_height_ * factor);
      } else {
        front_height = get_radius(cutoff_, amplitude_, sigma_front_height_);
      }

      rear_height = get_radius(cutoff_, amplitude_, sigma_rear_height_);

      double front_width = get_radius(cutoff_, amplitude_, sigma_front_width_);
      double rear_width = get_radius(cutoff_, amplitude_, sigma_rear_width_);
      double right_height = 0.0;
      if (use_passing_) {
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);
      }

      double height_diameter = std::max(front_height, rear_height) * 2.0;
      double width_diameter =
        (std::max(right_height, std::max(front_width, rear_width))) * 2.0;
      greater_side = std::max(height_diameter, width_diameter);
    }

    unsigned int width_cells =
      std::max(1, static_cast<int>(greater_side / res));
    unsigned int height_cells =
      std::max(1, static_cast<int>(greater_side / res));

    double cx = person.position.x, cy = person.position.y;

    double ox, oy;
    if (sin(angle) > 0) {
      oy = cy - rear_height;
    } else {
      oy = cy + (front_height - rear_height) * sin(angle) - rear_height;
    }

    if (cos(angle) >= 0) {
      ox = cx - rear_height;
    } else {
      ox = cx + (front_height - rear_height) * cos(angle) - rear_height;
    }

    unsigned int map_dx = 0u;
    unsigned int map_dy = 0u;
    if (!master_grid.worldToMap(ox, oy, map_dx, map_dy)) {
      continue;
    }
    const int dx = static_cast<int>(map_dx);
    const int dy = static_cast<int>(map_dy);

    int start_x = 0, start_y = 0, end_x = static_cast<int>(width_cells);
    int end_y = static_cast<int>(height_cells);
    const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
    const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
    const int clamp_max_i = std::min(max_i, size_x);
    const int clamp_max_j = std::min(max_j, size_y);

    if (static_cast<int>(start_x + dx) < min_i) {
      start_x = min_i - dx;
    }
    if (static_cast<int>(end_x + dx) > max_i) {
      end_x = std::max(0, max_i - dx);

    }
    if (dy < 0) {
      start_y = -dy;
    } else if (dy + height_cells > master_grid.getSizeInCellsY()) {
      end_y = std::max(0, static_cast<int>(master_grid.getSizeInCellsY()) - dy);
    }

    if (static_cast<int>(start_y + dy) < min_j) {
      start_y = min_j - dy;
    }
    if (static_cast<int>(end_y + dy) > max_j) {
      end_y = std::max(0, max_j - dy);

    }
    double bx = ox + res / 2, by = oy + res / 2;
    for (int i = start_x; i < end_x; i++) {
      const int cell_x = i + dx;
      if (cell_x < min_i || cell_x >= clamp_max_i) {
        continue;
      }
      for (int j = start_y; j < end_y; j++) {
        const int cell_y = j + dy;
        if (cell_y < min_j || cell_y >= clamp_max_j) {
          continue;
        }

        unsigned char old_cost = master_grid.getCost(cell_x, cell_y);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }

        double a;
        double a_right = 0.0;
        double x = bx + i * res;
        double y = by + j * res;
        if (mag < tolerance_vel_still_) {
          // PERSON STANDS STILL
          a = gaussian(
            x, y, cx, cy, amplitude_, sigma_when_still_,
            sigma_when_still_, 0);
        } else {

          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          // RIGHT SIDE
          if (use_passing_) {
            double diff_right =
              angles::shortest_angular_distance(angle_right, ma);
            if (fabs(diff_right) < M_PI / 2) {
              a_right = gaussian(
                x, y, cx, cy, amplitude_, sigma_right_height_,
                sigma_right_width_, angle_right);
            }
          }
          // FRONT
          if (fabs(diff) < M_PI / 2) {
            if (use_vel_factor_) {
              double factor = 1.0 + mag * speed_factor_;
              a = gaussian(
                x, y, cx, cy, amplitude_,
                sigma_front_height_ * factor, sigma_front_width_,
                angle);
            } else {
              a = gaussian(
                x, y, cx, cy, amplitude_, sigma_front_height_,
                sigma_front_width_, angle);
            }
          } else { // REAR
            a = gaussian(
              x, y, cx, cy, amplitude_, sigma_rear_height_,
              sigma_rear_width_,
              angle);              // 0

          }
          a = std::max(a, a_right);
        }
        if (a < cutoff_) {
          continue;
        }

        unsigned char cvalue = std::max((unsigned char)a, old_cost);
        master_grid.setCost(cell_x, cell_y, cvalue);
        const unsigned int index = master_grid.getIndex(cell_x, cell_y);
        if (index < grid.data.size()) {
          grid.data[index] = static_cast<int8_t>(a);
        }
      }
    }
  }
  if (publish_occgrid_) {
    grid_pub_->publish(grid);
  }

  // I had to add this because nobody is setting this
  // to true in Humble for some reason, so the controller_server
  // get stuck in an infinite loop waiting for the layers
  // to be current
  current_ = true;
}

double SocialLayer::gaussian(
  double x, double y, double x0, double y0, double A,
  double varx, double vary, double skew)
{
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx);
  double f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double SocialLayer::get_radius(double cutoff, double A, double var)
{
  return sqrt(-2 * var * log(cutoff / A));
}

} // namespace nav2_social_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_social_costmap_plugin::SocialLayer,
  nav2_costmap_2d::Layer)
