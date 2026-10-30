use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use costmap_planner_rs::extract::{costmap_to_grid, decode_costmap, inflate, Costmap, Grid};
use serde::Deserialize;

mod config;
mod loc;

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

/// shortcut: latitude/longitude are REP-103 meters (y/x), upgrade when a geo datum exists
#[derive(Debug, Clone, Copy, Deserialize)]
struct Goal {
    latitude: f64,
    longitude: f64,
}

impl Goal {
    fn xy(self) -> (f64, f64) {
        (self.longitude, self.latitude)
    }
}

fn heuristic_dist(a: (usize, usize), b: (usize, usize)) -> f64 {
    let dr = a.0.abs_diff(b.0) as f64;
    let dc = a.1.abs_diff(b.1) as f64;
    dr + dc
}

fn world_to_cell(costmap: &Costmap, x: f64, y: f64) -> (usize, usize) {
    let c = ((x - costmap.origin_x) / costmap.resolution).floor() as isize;
    let r = ((y - costmap.origin_y) / costmap.resolution).floor() as isize;
    (
        r.clamp(0, costmap.rows as isize - 1) as usize,
        c.clamp(0, costmap.cols as isize - 1) as usize,
    )
}

fn nearest_free(grid: &Grid, seed: (usize, usize)) -> Option<(usize, usize)> {
    if grid.at(seed.0, seed.1) < 253 {
        return Some(seed);
    }
    let mut seen = vec![false; grid.rows * grid.cols];
    let mut queue = VecDeque::new();
    queue.push_back(seed);
    seen[seed.0 * grid.cols + seed.1] = true;
    while let Some((r, c)) = queue.pop_front() {
        for (nr, nc) in [
            (r.wrapping_sub(1), c),
            (r + 1, c),
            (r, c.wrapping_sub(1)),
            (r, c + 1),
        ] {
            if nr >= grid.rows || nc >= grid.cols {
                continue;
            }
            let idx = nr * grid.cols + nc;
            if seen[idx] {
                continue;
            }
            seen[idx] = true;
            if grid.at(nr, nc) < 253 {
                return Some((nr, nc));
            }
            queue.push_back((nr, nc));
        }
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

fn build_trajectory(path: &[(usize, usize)], costmap: &Costmap) -> Vec<loc::Pose> {
    path.iter()
        .map(|&(r, c)| {
            loc::Pose::new(
                costmap.origin_x + (c as f64) * costmap.resolution,
                costmap.origin_y + (r as f64) * costmap.resolution,
                0.0,
            )
        })
        .collect()
}

// plan a path from the current pose to the goal using the given costmap
fn plan_once(
    msg: &Costmap,
    pose: &loc::Pose,
    goal: Goal,
) -> Option<Vec<loc::Pose>> {
    let mut grid = costmap_to_grid(msg);
    inflate(
        &mut grid,
        msg.resolution,
        config::INSCRIBED_RADIUS_M,
        config::DECAY_M,
    );
    let (px, py) = (pose.get_location().x(), pose.get_location().y());
    let (gx, gy) = goal.xy();
    let start = nearest_free(&grid, world_to_cell(msg, px, py))?;
    let goal_cell = nearest_free(&grid, world_to_cell(msg, gx, gy))?;
    println!("start={:?} goal={:?}", start, goal_cell);
    let path = astar(&grid, start, goal_cell)?;
    println!("path found: {} steps", path.len());
    Some(build_trajectory(&path, msg))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    println!("opening zenoh session pls work");
    let session = zenoh::open(zenoh::Config::default()).await?;
    let costmap_sub = session.declare_subscriber(config::COSTMAP_KEY).await?;
    let pose_sub = session.declare_subscriber(config::POSE_KEY).await?;
    let goal_sub = session.declare_subscriber(config::GOAL_KEY).await?;

    let mut pose: Option<loc::Pose> = None;
    let mut goal: Option<Goal> = None;

    loop {
        tokio::select! {
            result = pose_sub.recv_async() => {
                let Ok(sample) = result else {
                    eprintln!("pose subscription error: {:?}", result);
                    break;
                };
                match serde_json::from_slice::<loc::Pose>(&sample.payload().to_bytes()) {
                    Ok(p) => pose = Some(p),
                    Err(e) => eprintln!("failed to deserialize pose: {e}"),
                }
            }
            result = goal_sub.recv_async() => {
                let Ok(sample) = result else {
                    eprintln!("goal subscription error: {:?}", result);
                    break;
                };
                match serde_json::from_slice::<Goal>(&sample.payload().to_bytes()) {
                    Ok(g) if g.latitude.is_finite() && g.longitude.is_finite() => goal = Some(g),
                    Ok(_) => eprintln!("goal has non-finite latitude/longitude"),
                    Err(e) => eprintln!("failed to deserialize goal: {e}"),
                }
            }
            result = costmap_sub.recv_async() => {
                let Ok(sample) = result else {
                    eprintln!("costmap subscription error: {:?}", result);
                    break;
                };
                let (Some(pose), Some(goal)) = (&pose, goal) else {
                    continue;
                };
                let payload = sample.payload().to_bytes();
                let msg = match decode_costmap(&payload) {
                    Ok(msg) => msg,
                    Err(e) => {
                        eprintln!("failed to decode costmap: {e}");
                        continue;
                    }
                };
                if msg.rows == 0
                    || msg.cols == 0
                    || msg.rows.checked_mul(msg.cols) != Some(msg.data.len())
                    || !msg.resolution.is_finite()
                    || msg.resolution <= 0.0
                    || !msg.origin_x.is_finite()
                    || !msg.origin_y.is_finite()
                {
                    eprintln!("invalid costmap dimensions, data length, resolution, or origin");
                    continue;
                }
                let Some(trajectory) = plan_once(&msg, pose, goal) else {
                    eprintln!("no path");
                    continue;
                };
                let payload = serde_json::to_vec(&trajectory)?;
                session.put(config::TRAJECTORY_KEY, payload).await?;
            }
        }
    }

    Ok(())
}
