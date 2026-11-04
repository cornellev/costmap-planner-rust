//pub const COSTMAP_KEY: &str = "cev/perception/costmap_raw";
// The local grid is in the lidar frame. Pose and the goal are in odom.
pub const COSTMAP_KEY: &str = "cev/sensors/lidar/occupancy_grid/global";
pub const TRAJECTORY_KEY: &str = "cev/planning/ref_trajectory";
pub const POSE_KEY: &str = "cev/localization/pose";
pub const GOAL_KEY: &str = "cev/commands/goal";

/// Half of the merceves width. Pose centers inside this square of an obstacle collide.
pub const INSCRIBED_RADIUS_M: f64 = 0.31;
/// Past the lethal square, cell cost falls from 252 to 1 over this distance.
pub const DECAY_M: f64 = 0.5;

// Must match pure_pursuit/src/config.rs (WHEELBASE, MAX_STEERING_ANGLE).
pub const WHEELBASE_M: f64 = 0.66;
pub const MAX_STEER_RAD: f64 = 11.0_f64.to_radians();
