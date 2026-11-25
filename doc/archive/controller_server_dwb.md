# Controller Server for DWB

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 10.0
    odom_topic: /odom_nav
    
    min_x_velocity_threshold: 0.01
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.02
    
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.2
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::PositionGoalChecker"
      xy_goal_tolerance: 0.30
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false

      min_vel_x: 0.0
      max_vel_x: 0.25
      min_vel_y: 0.0
      max_vel_y: 0.0
      max_vel_theta: 0.5

      min_speed_xy: 0.0
      max_speed_xy: 0.25
      min_speed_theta: 0.0

      acc_lim_x: 0.3
      decel_lim_x: -0.3
      acc_lim_y: 0.0
      decel_lim_y: 0.0
      acc_lim_theta: 0.5
      decel_lim_theta: -0.5

      vx_samples: 10
      vy_samples: 1
      vtheta_samples: 20
      sim_time: 1.5
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2

      critics:
        - RotateToGoal
        - Oscillation
        - ObstacleFootprint
        - GoalAlign
        - PathAlign
        - PathDist
        - GoalDist

      ObstacleFootprint.scale: 0.02
      PathAlign.scale: 32.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 24.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```