pub const COSTMAP_KEY: &str = "cev/perception/costmap_raw";
pub const TRAJECTORY_KEY: &str = "cev/planning/ref_trajectory";
pub const POSE_KEY: &str = "cev/localization/pose";
pub const GOAL_KEY: &str = "cev/commands/goal";

/// Half of the merceves width. Pose centers inside this square of an obstacle collide.
pub const INSCRIBED_RADIUS_M: f64 = 0.31;
/// Past the lethal square, cell cost falls from 252 to 1 over this distance.
pub const DECAY_M: f64 = 0.5;
