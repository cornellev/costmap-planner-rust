use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
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

fn build_path_msg(path: &[(usize, usize)], costmap: &Costmap, clock: &Clock) -> Path {
    let mut path_out = Path::default();
    let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap_or((0, 0));
    path_out.header.stamp.sec = sec;
    path_out.header.stamp.nanosec = nanosec;
    path_out.header.frame_id = costmap.header.frame_id.clone();
    let resolution = costmap.metadata.resolution as f64;
    let origin_x = costmap.metadata.origin.position.x;
    let origin_y = costmap.metadata.origin.position.y;

    for &(r, c) in path {
        let mut pose = PoseStamped::default();
        pose.header = path_out.header.clone();
        pose.pose.position.x = origin_x + (c as f64) * resolution;
        pose.pose.position.y = origin_y + (r as f64) * resolution;
        pose.pose.position.z = 0.0;
        path_out.poses.push(pose);
    }

    path_out
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("astar_path_publisher")?;

    let publisher = node.create_publisher::<Path>("/astar_path")?;
    let logger = node.logger().clone();
    let clock = node.get_clock();

    let _subscription = node.create_subscription(
        "/costmap_raw",
        move |msg: Costmap| {
            let grid = costmap_to_grid(&msg);

            let start = match find_free_start(&grid) {
                Some(s) => s,
                None => {
                    log_warn!(&logger, "no free start found");
                    return;
                }
            };

            let goal = match find_free_goal_forward(&grid, start, 150) {
                Some(g) => g,
                None => {
                    log_warn!(&logger, "no forward goal found");
                    return;
                }
            };

            log_info!(&logger, "start={:?} goal={:?}", start, goal);
            let path = match astar(&grid, start, goal) {
                Some(p) => p,
                None => {
                    log_warn!(&logger, "found no path from {:?} to {:?}", start, goal);
                    return;
                }
            };

            log_info!(&logger, "path found: {} steps", path.len());
            let path_msg = build_path_msg(&path, &msg, &clock);
            publisher.publish(&path_msg).unwrap();
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}