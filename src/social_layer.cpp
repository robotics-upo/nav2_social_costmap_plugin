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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// END INCLUDES from socialLayer

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_social_costmap_plugin {

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and subscription to topics
void SocialLayer::onInitialize() {
  // START Subscription to topic
  ppl_sub_ = node_->create_subscription<people_msgs::msg::People>(
      "/people", rclcpp::SensorDataQoS(),
      std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
              "SocialLayer: subscribed to "
              "topic %s",
              ppl_sub_->get_topic_name());
  // END Subscription to topic

  // costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
  //    "/costmap_mat", 10);
  // costmap_pub_ = std::make_unique<nav2_costmap_2d::Costmap2DPublisher>(
  //     node_, layered_costmap_->getCostmap(),
  //     layered_costmap_->getGlobalFrameID(), "/social_costmap", True);
  // costmap_pub_->on_activate();

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
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>("social_grid", 1);
    grid_pub_->on_activate();
  }
}

void SocialLayer::get_parameters() {
  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(name_ + "." + "cutoff", cutoff_);
  node_->get_parameter(name_ + "." + "amplitude", amplitude_);
  node_->get_parameter(name_ + "." + "covariance_front_height",
                       sigma_front_height_);
  node_->get_parameter(name_ + "." + "covariance_front_width",
                       sigma_front_width_);
  node_->get_parameter(name_ + "." + "covariance_rear_height",
                       sigma_rear_height_);
  node_->get_parameter(name_ + "." + "covariance_rear_width",
                       sigma_rear_width_);
  node_->get_parameter(name_ + "." + "covariance_right_height",
                       sigma_right_height_);
  node_->get_parameter(name_ + "." + "covariance_right_width",
                       sigma_right_width_);
  node_->get_parameter(name_ + "." + "covariance_when_still",
                       sigma_when_still_);
  node_->get_parameter(name_ + "." + "use_passing", use_passing_);
  node_->get_parameter(name_ + "." + "use_vel_factor", use_vel_factor_);
  node_->get_parameter(name_ + "." + "speed_factor_multiplier", speed_factor_);
  node_->get_parameter(name_ + "." + "publish_occgrid", publish_occgrid_);
}

void SocialLayer::peopleCallback(
    const people_msgs::msg::People::SharedPtr msg) {
  ppl_message_mutex_.lock();
  people_list_ = *msg;
  ppl_message_mutex_.unlock();
}

void SocialLayer::updateBounds(double origin_x, double origin_y,
                               double origin_z, double *min_x, double *min_y,
                               double *max_x, double *max_y) {

  std::string global_frame =
      layered_costmap_
          ->getGlobalFrameID(); // Returns the global frame of the costmap
  transformed_people_.clear();  // empty the array
  int cnt_j = 0;

  for (unsigned int i = 0; i < people_list_.people.size(); i++) {
    people_msgs::msg::Person &person = people_list_.people[i];
    people_msgs::msg::Person tpt;
    geometry_msgs::msg::PointStamped pt, opt;

    pt.point.x = person.position.x;
    pt.point.y = person.position.y;
    pt.point.z = person.position.z;
    pt.header.frame_id = people_list_.header.frame_id;
    pt.header.stamp = people_list_.header.stamp;

    if (!tf_->canTransform(pt.header.frame_id, global_frame,
                           tf2_ros::fromMsg(pt.header.stamp))) {
      RCLCPP_INFO(node_->get_logger(),
                  "Social layer can't transform from %s to %s",
                  pt.header.frame_id.c_str(), global_frame.c_str());
      return;
    }

    // In general: tf_->transform(in_pose, out_pose, global_frame_,
    // transform_tolerance_);
    tf_->transform(pt, opt, global_frame);
    tpt.position.x = opt.point.x;
    tpt.position.y = opt.point.y;
    tpt.position.z = opt.point.z;

    pt.point.x += person.velocity.x;
    pt.point.y += person.velocity.y;
    pt.point.z += person.velocity.z;
    tf_->transform(pt, opt, global_frame);

    tpt.velocity.x = opt.point.x - tpt.position.x;
    tpt.velocity.y = opt.point.y - tpt.position.y;
    tpt.velocity.z = opt.point.z - tpt.position.z;

    cnt_j++;
    transformed_people_.push_back(
        tpt); // Adds a new element (tpt) at the end of the vector
  }

  std::list<people_msgs::msg::Person>::iterator p_it;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end();
       ++p_it) {
    people_msgs::msg::Person person = *p_it;

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
      if (use_passing_)
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);

      greater = std::max(
          front_height,
          std::max(rear_height,
                   std::max(right_height, std::max(front_width, rear_width))));
    }

    *min_x = std::min(*min_x, person.position.x - greater);
    *min_y = std::min(*min_y, person.position.y - greater);
    *max_x = std::max(*max_x, person.position.x + greater);
    *max_y = std::max(*max_y, person.position.y + greater);
  }
}

void SocialLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                              int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_) {
    return;
  }
  if (people_list_.people.size() == 0)
    return;

  if (cutoff_ >= amplitude_)
    return;

  get_parameters();

  std::list<people_msgs::msg::Person>::iterator p_it;
  nav2_costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
  double res = costmap->getResolution();

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = node_->get_clock()->now();
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();
  // grid.info.map_load_time
  grid.info.height = costmap->getSizeInCellsY();
  grid.info.width = costmap->getSizeInCellsX();
  grid.info.resolution = res;
  grid.info.origin.position.x = costmap->getOriginX();
  grid.info.origin.position.y = costmap->getOriginY();
  grid.info.origin.orientation.w = 1.0;
  std::vector<int8_t> vect((grid.info.height * grid.info.width), 0);
  grid.data = vect;

  for (p_it = transformed_people_.begin(); p_it != transformed_people_.end();
       ++p_it) {
    people_msgs::msg::Person person = *p_it;
    double mag = sqrt(person.velocity.x * person.velocity.x +
                      person.velocity.y * person.velocity.y);
    double angle = atan2(person.velocity.y, person.velocity.x);
    double angle_right = angle - 1.57; // 1.51;
    double radius = get_radius(cutoff_, amplitude_, sigma_when_still_);
    double front_height = radius;
    double rear_height = radius;
    double greater_side = radius + radius;
    if (mag >= tolerance_vel_still_) {
      if (use_vel_factor_) {
        double factor = 1.0 + mag * speed_factor_;
        front_height =
            get_radius(cutoff_, amplitude_, sigma_front_height_ * factor);
      } else
        front_height = get_radius(cutoff_, amplitude_, sigma_front_height_);

      rear_height = get_radius(cutoff_, amplitude_, sigma_rear_height_);

      double front_width = get_radius(cutoff_, amplitude_, sigma_front_width_);
      double rear_width = get_radius(cutoff_, amplitude_, sigma_rear_width_);
      double right_height = 0.0;
      if (use_passing_)
        right_height = get_radius(cutoff_, amplitude_, sigma_right_height_);

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
    if (sin(angle) > 0)
      oy = cy - rear_height;
    else
      oy = cy + (front_height - rear_height) * sin(angle) - rear_height;

    if (cos(angle) >= 0)
      ox = cx - rear_height;
    else
      ox = cx + (front_height - rear_height) * cos(angle) - rear_height;

    int dx, dy;
    // Convert from world coordinates to map coordinates without checking for
    // legal bounds world x, world y, map x, map y
    costmap->worldToMapNoBounds(ox, oy, dx, dy);

    int start_x = 0, start_y = 0, end_x = width_cells, end_y = height_cells;
    if (dx < 0)
      start_x = -dx;
    else if (dx + width_cells > costmap->getSizeInCellsX())
      end_x = std::max(0, static_cast<int>(costmap->getSizeInCellsX()) - dx);

    if (static_cast<int>(start_x + dx) < min_i)
      start_x = min_i - dx;
    if (static_cast<int>(end_x + dx) > max_i)
      end_x = max_i - dx;

    if (dy < 0)
      start_y = -dy;
    else if (dy + height_cells > costmap->getSizeInCellsY())
      end_y = std::max(0, static_cast<int>(costmap->getSizeInCellsY()) - dy);

    if (static_cast<int>(start_y + dy) < min_j)
      start_y = min_j - dy;
    if (static_cast<int>(end_y + dy) > max_j)
      end_y = max_j - dy;

    double bx = ox + res / 2, by = oy + res / 2;
    for (int i = start_x; i < end_x; i++) {
      for (int j = start_y; j < end_y; j++) {
        unsigned char old_cost = costmap->getCost(i + dx, j + dy);
        if (old_cost == nav2_costmap_2d::NO_INFORMATION)
          continue;

        double a;
        double a_right = 0.0;
        double x = bx + i * res;
        double y = by + j * res;
        if (mag < tolerance_vel_still_) {
          // PERSON STANDS STILL
          a = gaussian(x, y, cx, cy, amplitude_, sigma_when_still_,
                       sigma_when_still_, 0);
        } else {

          double ma = atan2(y - cy, x - cx);
          double diff = angles::shortest_angular_distance(angle, ma);
          // RIGHT SIDE
          if (use_passing_) {
            double diff_right =
                angles::shortest_angular_distance(angle_right, ma);
            if (fabs(diff_right) < M_PI / 2) {
              a_right = gaussian(x, y, cx, cy, amplitude_, sigma_right_height_,
                                 sigma_right_width_, angle_right);
            }
          }
          // FRONT
          if (fabs(diff) < M_PI / 2) {
            if (use_vel_factor_) {
              double factor = 1.0 + mag * speed_factor_;
              a = gaussian(x, y, cx, cy, amplitude_,
                           sigma_front_height_ * factor, sigma_front_width_,
                           angle);
            } else
              a = gaussian(x, y, cx, cy, amplitude_, sigma_front_height_,
                           sigma_front_width_, angle);
          } else // REAR
            a = gaussian(x, y, cx, cy, amplitude_, sigma_rear_height_,
                         sigma_rear_width_,
                         angle); // 0

          a = std::max(a, a_right);
        }
        if (a < cutoff_)
          continue;

        unsigned char cvalue = (unsigned char)a;
        costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
        unsigned int index = costmap->getIndex(i + dx, j + dy);
        grid.data[index] = (unsigned int)a;
      }
    }
  }
  if (publish_occgrid_)
    grid_pub_->publish(grid);
}

double SocialLayer::gaussian(double x, double y, double x0, double y0, double A,
                             double varx, double vary, double skew) {
  double dx = x - x0, dy = y - y0;
  double h = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);
  double mx = cos(angle - skew) * h;
  double my = sin(angle - skew) * h;
  double f1 = pow(mx, 2.0) / (2.0 * varx);
  double f2 = pow(my, 2.0) / (2.0 * vary);
  return A * exp(-(f1 + f2));
}

double SocialLayer::get_radius(double cutoff, double A, double var) {
  return sqrt(-2 * var * log(cutoff / A));
}

} // namespace nav2_social_costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_social_costmap_plugin::SocialLayer,
                       nav2_costmap_2d::Layer)