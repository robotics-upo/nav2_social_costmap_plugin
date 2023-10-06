# Nav2 social costmap plugin

Plugin-based layer for social navigation in the Nav2 system of ROS2 (tested in ROS2 Humble Distro).

It is based on the outdated [social_navigation_layers](http://wiki.ros.org/social_navigation_layers) package of ROS1 Melodic.
In our case, the ProxemicsLayer and the PassingLayer have been joined in a single layer. 


## Dependencies

- nav2-costmap-2d
- nav_msgs
- people_msgs. At the moment of this development, people_msgs were not still available to be installed from the apt ros-foxy package server. You can get the package from here: https://github.com/wg-perception/people/tree/ros2. Please, copy it and put it in your workspace.


## Subscribed Topic 

- */people* [people_msgs/People]. The people around the robot.

## Published Topic

- *(global or local)_costmap/social_grid [nav_msgs/OccupancyGrid]. Occupancy grid with the social costs employed. Only active if parameter `publish_occgrid` is True.   

## Parameter configuration


  * `enabled` (bool, default: True). Whether to apply this plugin or not.
  * `cutoff` (double, default: 5.0). Smallest cost value to publish on the costmap. 
  * `amplitude` (double, default: 255.0). Amplitude of adjustments at peak. Maximum cost on the person center.
  * `covariance_front_height` (double, default: 0.25). Covariance of the ordinate axis of the Gaussian at the person heading. 
  * `covariance_front_width` (double, default: 0.25). Covariance of the abscissa axis of the Gaussian at the person heading. 
  * `covariance_rear_height` (double, default: 0.25). Covariance of the ordinate axis of the Gaussian at the person's rear. 
  * `covariance_rear_width` (double, default: 0.25). Covariance of the abscissa axis of the Gaussian at the person's rear.
  * `covariance_when_still` (double, default: 0.25). Covariance employed to form a circular cost around the person when she is not moving. 
  * `use_vel_factor` (bool, default: True). Whether to use the current person velocity and the `speed_factor_multiplier` to modify the Gaussian at the person's front or not.  
  * `speed_factor_multiplier` (double, default: 5.0). Factor with which to scale the velocity.
  * `use_passing` (bool, default: True). Whether to use another Gaussian in the right side of the person in order to ease the passing maneuvers, for instance in corridors.
  * `covariance_right_height` (double, default: 0.25). Covariance of the ordinate axis of the Gaussian at the person's right side. 
  * `covariance_right_width` (double, default: 0.25). Covariance of the abscissa axis of the Gaussian at the person's right side'.
  * `publish_occgrid` (bool, default: False). Whether to publish an OccupancyGrid with the only the social costs or not. 

NOTE: To be used as the previous ProxemicsLayer of ROS1, the user must set `use_passing` to False, `use_vel_factor` to True, and to use the same value for the parameters `covariance_front_height`, `covariance_front_width`, `covariance_rear_height`, `covariance_rear_width` and `covariance_when_still`. 

## Example of use in Nav2

In the XML configuration file of the Nav2 we must add (in the local_costmap and/or global_costmap section):

```
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: [(other plugins..), "social_layer"]
      social_layer:
        plugin: "nav2_social_costmap_plugin::SocialLayer"
        enabled: True
        cutoff: 10.0
        amplitude: 255.0
        publish_occgrid: False
        use_passing: True
        use_vel_factor: True
        speed_factor_multiplier: 5.0
        covariance_front_height: 0.4
        covariance_front_width: 0.25
        covariance_rear_height: 0.25
        covariance_rear_width: 0.25
        covariance_right_height: 0.3
        covariance_right_width: 0.2
        covariance_when_still: 0.25
```
