use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fs::OpenOptions;
use std::io::Write;
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};
use rclrs::*;
use nav2_msgs::msg::Costmap;
use nav_msgs::msg::Path;
use geometry_msgs::msg::PoseStamped;
use costmap_planner_rs::extract::{costmap_to_grid, Grid};

#[derive(Copy, Clone, PartialEq)]
struct HeapItem {
    f_score: f64,
    pos: (usize, usize),
}

impl Eq for HeapItem {}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn heuristic_dist(a: (usize, usize), b: (usize, usize)) -> f64 {
    let dr = a.0.abs_diff(b.0) as f64;
    let dc = a.1.abs_diff(b.1) as f64;
    dr + dc
}

/*
fn find_free_start(grid: &Grid) -> Option<(usize, usize)> {
    let rows = grid.rows as isize;
    let cols = grid.cols as isize;
    let center_r = rows / 2;
    let center_c = cols / 2;

    for radius in 1..rows.min(cols) {
        for dr in -radius..radius {
            for dc in -radius..radius {
                let r = center_r + dr;
                let c = center_c + dc;
                if r >= 0 && r < rows && c >= 0 && c < cols {
                    if grid.at(r as usize, c as usize) < 200 {
                        return Some((r as usize, c as usize));
                    }
                }
            }
        }
    }
    None
}

*/

fn world_to_cell(x: f64, y: f64, costmap: &Costmap) -> Option<(usize, usize)> {
    let resolution = costmap.metadata.resolution as f64;
    let column = ((x - costmap.metadata.origin.position.x) / resolution).floor();
    let row = ((y - costmap.metadata.origin.position.y) / resolution).floor();
    if row < 0.0
        || column < 0.0
        || row >= costmap.metadata.size_y as f64
        || column >= costmap.metadata.size_x as f64
    {
        return None;
    }
    Some((row as usize, column as usize))
}

fn local_goal_cell(
    grid: &Grid,
    start_world: (f64, f64),
    goal_world: (f64, f64),
    costmap: &Costmap,
) -> Option<(usize, usize)> {
    if let Some(goal) = world_to_cell(goal_world.0, goal_world.1, costmap) {
        return Some(goal);
    }

    // rolling window logic, ignore it no goal cell (probably would become centerlines)
    let dx = goal_world.0 - start_world.0;
    let dy = goal_world.1 - start_world.1;
    let distance = (dx * dx + dy * dy).sqrt();
    if distance <= f64::EPSILON {
        return None;
    }
    let step_x = dx / distance;
    let step_y = dy / distance;
    let max_distance = (costmap.metadata.size_x.min(costmap.metadata.size_y) as f64
        * costmap.metadata.resolution as f64)
        * 0.45;

    // stupid code for rolling, scrapped, ignore it
    let mut travel = max_distance;
    while travel > 0.0 {
        let candidate = (
            start_world.0 + step_x * travel,
            start_world.1 + step_y * travel,
        );
        if let Some(cell) = world_to_cell(candidate.0, candidate.1, costmap) {
            if grid.at(cell.0, cell.1) < 253 {
                return Some(cell);
            }
        }
        travel -= costmap.metadata.resolution as f64;
    }
    None
}

/*
fn find_free_goal_forward(grid: &Grid, start: (usize, usize), target_dist: isize,) -> Option<(usize, usize)> {
    let rows = grid.rows as isize;
    let cols = grid.cols as isize;
    let r = start.0 as isize;
    let c = start.1 as isize;

    let mut dist = target_dist;
    while dist > 20 {
        let gc = c + dist;
        if gc >= 0 && gc < cols && grid.at(r as usize, gc as usize) < 253 {
            return Some((r as usize, gc as usize));
        }
        dist -= 1;
    }

    let mut dist = target_dist;
    while dist > 20 {
        let half_width = dist / 4;
        for dr in -half_width..=half_width {
            let gr = r + dr;
            let gc = c + dist;
            if gr >= 0 && gr < rows && gc >= 0 && gc < cols {
                if grid.at(gr as usize, gc as usize) < 253 {
                    return Some((gr as usize, gc as usize));
                }
            }
        }
        dist -= 1;
    }

    None
}

*/

fn astar(grid: &Grid, start: (usize, usize), goal: (usize, usize)) -> Option<Vec<(usize, usize)>> {
    let mut open_set = BinaryHeap::new();
    open_set.push(HeapItem { f_score: 0.0, pos: start });

    let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();
    let mut g_score: HashMap<(usize, usize), f64> = HashMap::new();
    g_score.insert(start, 0.0);

    while let Some(HeapItem { pos: current, .. }) = open_set.pop() {
        if current == goal {
            let mut path = vec![current];
            let mut node = current;
            while let Some(&prev) = came_from.get(&node) {
                path.push(prev);
                node = prev;
            }
            path.reverse();
            return Some(path);
        }

        let (cr, cc) = (current.0 as isize, current.1 as isize);
        let candidates = [(cr + 1, cc), (cr - 1, cc), (cr, cc + 1), (cr, cc - 1)];

        for (nr, nc) in candidates {
            if nr < 0 || nr >= grid.rows as isize || nc < 0 || nc >= grid.cols as isize {
                continue;
            }
            let n = (nr as usize, nc as usize);
            let cost = grid.at(n.0, n.1) as f64;
            if cost >= 253.0 {
                continue;
            }

            let tentative_g = g_score[&current] + 1.0 + cost / 255.0;
            let is_better = match g_score.get(&n) {
                Some(&existing) => tentative_g < existing,
                None => true,
            };

            if is_better {
                came_from.insert(n, current);
                g_score.insert(n, tentative_g);
                let f_score = tentative_g + heuristic_dist(n, goal);
                open_set.push(HeapItem { f_score, pos: n });
            }
        }
    }

    None
}

fn path_to_world_points(path: &[(usize, usize)], costmap: &Costmap) -> Vec<(f64, f64)> {
    let resolution = costmap.metadata.resolution as f64;
    let origin_x = costmap.metadata.origin.position.x;
    let origin_y = costmap.metadata.origin.position.y;

    path.iter()
        .map(|&(r, c)| {
            (
                origin_x + (c as f64 + 0.5) * resolution,
                origin_y + (r as f64 + 0.5) * resolution,
            )
        })
        .collect()
}

fn simplify_path(points: &[(f64, f64)], min_step_sq: f64) -> Vec<(f64, f64)> {
    if points.is_empty() {
        return Vec::new();
    }
    if points.len() <= 2 {
        return points.to_vec();
    }

    let mut simplified = vec![points[0]];
    for &point in &points[1..points.len() - 1] {
        let last = *simplified.last().unwrap();
        let dx = point.0 - last.0;
        let dy = point.1 - last.1;
        if dx * dx + dy * dy >= min_step_sq {
            simplified.push(point);
        }
    }
    simplified.push(*points.last().unwrap());
    simplified
}

fn smooth_path(points: &[(f64, f64)], window: usize) -> Vec<(f64, f64)> {
    if points.len() <= 2 {
        return points.to_vec();
    }

    let mut smoothed = Vec::with_capacity(points.len());
    let radius = window / 2;
    for i in 0..points.len() {
        let start = i.saturating_sub(radius);
        let end = (i + radius + 1).min(points.len());
        let mut x_sum = 0.0;
        let mut y_sum = 0.0;
        let mut count = 0usize;

        for j in start..end {
            x_sum += points[j].0;
            y_sum += points[j].1;
            count += 1;
        }

        let inv = 1.0 / count as f64;
        smoothed.push((x_sum * inv, y_sum * inv));
    }

    smoothed
}

fn resample_path(points: &[(f64, f64)], max_step: f64) -> Vec<(f64, f64)> {
    if points.len() < 2 {
        return points.to_vec();
    }

    let mut sampled = vec![points[0]];
    for window in points.windows(2) {
        let p0 = window[0];
        let p1 = window[1];
        let dx = p1.0 - p0.0;
        let dy = p1.1 - p0.1;
        let seg_len = (dx * dx + dy * dy).sqrt();
        if seg_len <= f64::EPSILON {
            continue;
        }

        let steps = (seg_len / max_step).ceil() as usize;
        for step in 1..=steps {
            let t = step as f64 / steps as f64;
            let x = p0.0 + dx * t;
            let y = p0.1 + dy * t;
            if sampled.last().map_or(true, |last| {
                (last.0 - x).abs() > 1e-9 || (last.1 - y).abs() > 1e-9
            }) {
                sampled.push((x, y));
            }
        }
    }

    if sampled.last() != points.last() {
        sampled.push(*points.last().unwrap());
    }

    sampled
}

fn heading_from_points(prev: (f64, f64), curr: (f64, f64)) -> f64 {
    let dx = curr.0 - prev.0;
    let dy = curr.1 - prev.1;
    if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
        return 0.0;
    }
    dy.atan2(dx)
}

fn quaternion_from_yaw(yaw: f64) -> geometry_msgs::msg::Quaternion {
    let half = yaw / 2.0;
    geometry_msgs::msg::Quaternion {
        x: 0.0,
        y: 0.0,
        z: half.sin(),
        w: half.cos(),
    }
}

fn build_path_msg(path: &[(usize, usize)], costmap: &Costmap, clock: &Clock) -> Path {
    let mut path_out = Path::default();
    let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap_or((0, 0));
    path_out.header.stamp.sec = sec;
    path_out.header.stamp.nanosec = nanosec;
    path_out.header.frame_id = costmap.header.frame_id.clone();

    let world_points = path_to_world_points(path, costmap);
    let min_step_sq = (costmap.metadata.resolution as f64 * 1.5).powi(2);
    let simplified_points = simplify_path(&world_points, min_step_sq);
    let smoothed_points = smooth_path(&simplified_points, 5);
    let sample_step = costmap.metadata.resolution as f64 * 2.0;
    let sampled_points = resample_path(&smoothed_points, sample_step);

    for (i, (x, y)) in sampled_points.iter().enumerate() {
        let mut pose = PoseStamped::default();
        pose.header = path_out.header.clone();
        pose.pose.position.x = *x;
        pose.pose.position.y = *y;
        pose.pose.position.z = 0.0;

        let yaw = if i == 0 {
            if sampled_points.len() > 1 {
                heading_from_points(sampled_points[0], sampled_points[1])
            } else {
                0.0
            }
        } else {
            heading_from_points(sampled_points[i - 1], sampled_points[i])
        };
        pose.pose.orientation = quaternion_from_yaw(yaw);
        path_out.poses.push(pose);
    }

    path_out
}

fn write_timing_row(timing_file: &Arc<Mutex<std::fs::File>>, row: &str) {
    if let Ok(mut file) = timing_file.lock() {
        let _ = writeln!(file, "{}", row);
        let _ = file.flush();
    }
}

fn timestamp_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or(0)
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("astar_path_publisher")?;

    let publisher = node.create_publisher::<Path>("/astar_path")?;
    let trajectory_publisher = node.create_publisher::<Path>("/astar_trajectory")?;
    let logger = node.logger().clone();
    let clock = node.get_clock();
    let timing_file = Arc::new(Mutex::new(
        OpenOptions::new()
            .create(true)
            .append(true)
            .open("astar_timing.csv")
            .expect("unable to open astar_timing.csv"),
    ));
    if let Ok(mut file) = timing_file.lock() {
        let _ = file.write_all(
            b"timestamp_ns,rows,cols,start_row,start_col,goal_row,goal_col,path_steps,astar_ms,found\n",
        );
    }

    let pose_state = Arc::new(Mutex::new(None::<(f64, f64)>));
    let goal_state = Arc::new(Mutex::new(None::<(f64, f64)>));

    let pose_for_subscription = Arc::clone(&pose_state);
    let _pose_subscription = node.create_subscription(
        "/sim_pose",
        move |msg: PoseStamped| {
            if let Ok(mut pose) = pose_for_subscription.lock() {
                *pose = Some((msg.pose.position.x, msg.pose.position.y));
            }
        },
    )?;

    let goal_for_subscription = Arc::clone(&goal_state);
    let _goal_subscription = node.create_subscription(
        "/goal_pose",
        move |msg: PoseStamped| {
            if let Ok(mut goal) = goal_for_subscription.lock() {
                *goal = Some((msg.pose.position.x, msg.pose.position.y));
            }
        },
    )?;

    let pose_state_for_costmap = Arc::clone(&pose_state);
    let goal_state_for_costmap = Arc::clone(&goal_state);
    let timing_file_for_costmap = Arc::clone(&timing_file);

    let _subscription = node.create_subscription(
        "/costmap_raw",
        move |msg: Costmap| {
            let grid = costmap_to_grid(&msg);

            let start_world = match pose_state_for_costmap.lock().ok().and_then(|pose| *pose) {
                Some(pose) => pose,
                None => {
                    log_warn!(&logger, "waiting for simulator start pose");
                    return;
                }
            };

            let goal_world = match goal_state_for_costmap.lock().ok().and_then(|goal| *goal) {
                Some(goal) => goal,
                None => {
                    log_warn!(&logger, "waiting for simulator map goal");
                    return;
                }
            };

            let start = match world_to_cell(start_world.0, start_world.1, &msg) {
                Some(cell) => cell,
                None => {
                    log_warn!(&logger, "simulator start is outside rolling costmap");
                    return;
                }
            };
            let goal = match local_goal_cell(&grid, start_world, goal_world, &msg) {
                Some(cell) => cell,
                None => {
                    log_warn!(&logger, "no free local goal toward simulator goal");
                    return;
                }
            };

            // problematic
            if grid.at(start.0, start.1) >= 253 || grid.at(goal.0, goal.1) >= 253 {
                log_warn!(&logger, "start or goal is occupied in inflated costmap");
                return;
            }

            log_info!(&logger, "start={:?} goal={:?}", start, goal);
            let planning_started = Instant::now();
            let path = match astar(&grid, start, goal) {
                Some(p) => p,
                None => {
                    let elapsed_ms = planning_started.elapsed().as_secs_f64() * 1000.0;
                    write_timing_row(
                        &timing_file_for_costmap,
                        &format!(
                            "{},{},{},{},{},{},{},0,{:.6},false",
                            timestamp_ns(),
                            grid.rows,
                            grid.cols,
                            start.0,
                            start.1,
                            goal.0,
                            goal.1,
                            elapsed_ms
                        ),
                    );
                    log_warn!(
                        &logger,
                        "found no path from {:?} to {:?}; astar_ms={:.3}",
                        start,
                        goal,
                        planning_started.elapsed().as_secs_f64() * 1000.0
                    );
                    return;
                }
            };

            let elapsed_ms = planning_started.elapsed().as_secs_f64() * 1000.0;
            write_timing_row(
                &timing_file_for_costmap,
                &format!(
                    "{},{},{},{},{},{},{},{},{:.6},true",
                    timestamp_ns(),
                    grid.rows,
                    grid.cols,
                    start.0,
                    start.1,
                    goal.0,
                    goal.1,
                    path.len(),
                    elapsed_ms
                ),
            );
            log_info!(
                &logger,
                "path found: {} steps; astar_ms={:.3}",
                path.len(),
                elapsed_ms
            );
            let path_msg = build_path_msg(&path, &msg, &clock);
            publisher.publish(&path_msg).unwrap();
            trajectory_publisher.publish(&path_msg).unwrap();
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}