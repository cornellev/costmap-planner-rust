use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use costmap_planner_rs::extract::{costmap_to_grid, decode_costmap, Costmap, Grid};

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

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    println!("opening zenoh session pls work");
    let session = zenoh::open(zenoh::Config::default()).await?;
    let costmap_sub = session.declare_subscriber(config::COSTMAP_KEY).await?;

    loop {
        let sample = costmap_sub.recv_async().await?;
        let payload = sample.payload().to_bytes();
        let msg = match decode_costmap(&payload) {
            Ok(msg) => msg,
            Err(e) => {
                eprintln!("failed to decode costmap: {e}");
                continue;
            }
        };

        // just basic sanity checks on the costmap to see if it's like actually valid lol
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

        let grid = costmap_to_grid(&msg); // just did a basic ahh conversion lol

        let start = match find_free_start(&grid) {
            Some(s) => s,
            None => {
                eprintln!("no free start found");
                continue;
            }
        };

        // target dist is just a guess, we can adjust this later
        let goal = match find_free_goal_forward(&grid, start, 150) {
            Some(g) => g,
            None => {
                eprintln!("no forward goal found");
                continue;
            }
        };

        println!("start={:?} goal={:?}", start, goal);
        let path = match astar(&grid, start, goal) {
            Some(p) => p,
            None => {
                eprintln!("found no path from {:?} to {:?}", start, goal);
                continue;
            }
        };

        // out out out
        println!("path found: {} steps", path.len());
        let trajectory = build_trajectory(&path, &msg);
        let payload = serde_json::to_vec(&trajectory)?;
        session.put(config::TRAJECTORY_KEY, payload).await?;
    }
}
