use costmap_planner_rs::extract::{Costmap, Grid, costmap_to_grid, decode_costmap, inflate};
use serde::Deserialize;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::f64::consts::PI;

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
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
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
    open_set.push(HeapItem {
        f_score: 0.0,
        pos: start,
    });

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
            // Cell center. The corner sits a full cell off a pose on the far edge.
            loc::Pose::new(
                costmap.origin_x + (c as f64 + 0.5) * costmap.resolution,
                costmap.origin_y + (r as f64 + 0.5) * costmap.resolution,
                0.0,
            )
        })
        .collect()
}

fn min_turn_radius() -> f64 {
    config::WHEELBASE_M / config::MAX_STEER_RAD.tan()
}

fn wrap_angle(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(2.0 * PI);
    if wrapped > PI {
        wrapped - 2.0 * PI
    } else {
        wrapped
    }
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

fn normalize(v: (f64, f64)) -> Option<(f64, f64)> {
    let len = (v.0 * v.0 + v.1 * v.1).sqrt();
    if len < 1e-9 {
        None
    } else {
        Some((v.0 / len, v.1 / len))
    }
}

fn lethal_at(grid: &Grid, map: &Costmap, x: f64, y: f64) -> bool {
    let res = map.resolution;
    if !(res > 0.0) || !x.is_finite() || !y.is_finite() {
        return true;
    }
    let c = ((x - map.origin_x) / res).floor();
    let r = ((y - map.origin_y) / res).floor();
    if r < 0.0 || c < 0.0 || r >= grid.rows as f64 || c >= grid.cols as f64 {
        return true;
    }
    grid.at(r as usize, c as usize) >= 253
}

// shortcut: half-cell samples can miss a corner clip, switch to a supercover walk if a shortcut cuts a wall.
fn segment_clear(a: (f64, f64), b: (f64, f64), grid: &Grid, map: &Costmap) -> bool {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let len = (dx * dx + dy * dy).sqrt();
    let step = (map.resolution * 0.5).max(1e-3);
    let n = (len / step).ceil().max(1.0) as usize;
    for i in 0..=n {
        let t = i as f64 / n as f64;
        if lethal_at(grid, map, a.0 + dx * t, a.1 + dy * t) {
            return false;
        }
    }
    true
}

fn shortcut(points: &[(f64, f64)], grid: &Grid, map: &Costmap) -> Vec<(f64, f64)> {
    // shortcut: farthest-visible scan is quadratic in path cells. Upgrade if a long blocked path misses the 1 Hz replan.
    if points.len() < 3 {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    let mut i = 0;
    while i < points.len() - 1 {
        let mut j = points.len() - 1;
        while j > i + 1 && !segment_clear(points[i], points[j], grid, map) {
            j -= 1;
        }
        out.push(points[j]);
        i = j;
    }
    out
}

fn push_xy(out: &mut Vec<(f64, f64)>, p: (f64, f64)) {
    if out.last().is_none_or(|q| dist(*q, p) > 1e-6) {
        out.push(p);
    }
}

enum Fillet {
    Keep,
    Arc(Vec<(f64, f64)>),
    Reject,
}

fn sample_arc(
    center: (f64, f64),
    start: f64,
    sweep: f64,
    radius: f64,
    spacing: f64,
) -> Vec<(f64, f64)> {
    let n = ((radius * sweep.abs()) / spacing.max(1e-3)).ceil().max(1.0) as usize;
    (0..=n)
        .map(|i| {
            let a = start + sweep * (i as f64 / n as f64);
            (center.0 + radius * a.cos(), center.1 + radius * a.sin())
        })
        .collect()
}

fn outward_unit(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<(f64, f64)> {
    let u1 = normalize((a.0 - b.0, a.1 - b.1))?;
    let u2 = normalize((c.0 - b.0, c.1 - b.1))?;
    normalize((-u1.0 - u2.0, -u1.1 - u2.1))
}

fn try_fillet(
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
    radius: f64,
    grid: &Grid,
    map: &Costmap,
) -> Fillet {
    let Some(d1) = normalize((b.0 - a.0, b.1 - a.1)) else {
        return Fillet::Keep;
    };
    let Some(d2) = normalize((c.0 - b.0, c.1 - b.1)) else {
        return Fillet::Keep;
    };
    let delta = wrap_angle(d2.1.atan2(d2.0) - d1.1.atan2(d1.0));
    if delta.abs() < 1.0_f64.to_radians() {
        return Fillet::Keep;
    }
    // tan(delta/2) blows up as the turn flattens into a U.
    if delta.abs() > 150.0_f64.to_radians() {
        return Fillet::Reject;
    }
    let t = radius * (delta.abs() * 0.5).tan();
    if !t.is_finite() || t > dist(a, b) + 1e-4 || t > dist(b, c) + 1e-4 {
        return Fillet::Reject;
    }
    let t1 = (b.0 - d1.0 * t, b.1 - d1.1 * t);
    let normal = if delta > 0.0 {
        (-d1.1, d1.0)
    } else {
        (d1.1, -d1.0)
    };
    let center = (t1.0 + normal.0 * radius, t1.1 + normal.1 * radius);
    let start = (t1.1 - center.1).atan2(t1.0 - center.0);
    let step = (map.resolution * 0.5).max(1e-3);
    if sample_arc(center, start, delta, radius, step)
        .iter()
        .any(|&p| lethal_at(grid, map, p.0, p.1))
        || !segment_clear(a, b, grid, map)
        || !segment_clear(b, c, grid, map)
    {
        return Fillet::Reject;
    }
    // 0.5 m chords sit a few millimeters off the arc, inside the lookahead.
    Fillet::Arc(sample_arc(center, start, delta, radius, 0.5))
}

fn fillet_corner(
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
    radius: f64,
    grid: &Grid,
    map: &Costmap,
) -> Option<Vec<(f64, f64)>> {
    match try_fillet(a, b, c, radius, grid, map) {
        Fillet::Arc(arc) => return Some(arc),
        Fillet::Keep => return None,
        Fillet::Reject => {}
    }
    let Some(dir) = outward_unit(a, b, c) else {
        eprintln!(
            "turn is tighter than {:.0}°",
            config::MAX_STEER_RAD.to_degrees()
        );
        return None;
    };
    let step = map.resolution.max(1e-3);
    let mut shift = step;
    while shift <= radius + 1e-9 {
        let moved = (b.0 + dir.0 * shift, b.1 + dir.1 * shift);
        if let Fillet::Arc(arc) = try_fillet(a, moved, c, radius, grid, map) {
            return Some(arc);
        }
        shift += step;
    }
    eprintln!(
        "turn is tighter than {:.0}°",
        config::MAX_STEER_RAD.to_degrees()
    );
    None
}

fn fillet(points: &[(f64, f64)], radius: f64, grid: &Grid, map: &Costmap) -> Vec<(f64, f64)> {
    if points.len() < 3 || !(radius > 0.0) {
        return points.to_vec();
    }
    let mut out = vec![points[0]];
    let mut anchor = points[0];
    for i in 1..points.len() - 1 {
        if let Some(arc) = fillet_corner(anchor, points[i], points[i + 1], radius, grid, map) {
            for p in arc {
                push_xy(&mut out, p);
            }
            anchor = *out.last().unwrap();
        } else {
            push_xy(&mut out, points[i]);
            anchor = points[i];
        }
    }
    push_xy(&mut out, *points.last().unwrap());
    out
}

fn smooth_path(points: &[(f64, f64)], grid: &Grid, map: &Costmap) -> Vec<(f64, f64)> {
    fillet(&shortcut(points, grid, map), min_turn_radius(), grid, map)
}

/// Pose is inside the start cell, not on its center. A straight plan then
/// looks a few centimeters to one side and pure pursuit holds a steer.
/// Only slide by a sub-cell amount; a nearest_free jump must stay put.
fn snap_start_to_pose(path: Vec<loc::Pose>, pose: &loc::Pose, resolution: f64) -> Vec<loc::Pose> {
    let Some(first) = path.first() else {
        return path;
    };
    let dx = pose.get_location().x() - first.get_location().x();
    let dy = pose.get_location().y() - first.get_location().y();
    if dx * dx + dy * dy > resolution * resolution {
        return path;
    }
    path.into_iter()
        .map(|p| {
            let loc = p.get_location();
            loc::Pose::new(loc.x() + dx, loc.y() + dy, p.get_heading())
        })
        .collect()
}

// plan a path from the current pose to the goal using the given costmap
fn plan_once(msg: &Costmap, pose: &loc::Pose, goal: Goal) -> Option<Vec<loc::Pose>> {
    let mut grid = costmap_to_grid(msg);
    // shortcut: 255 is an unscanned lidar shadow, not a wall. inflate() seeds every
    // cost >= 253, so leaving 255 seals the corridor. Upgrade when the map is localized.
    for cost in &mut grid.data {
        if *cost == 255 {
            *cost = 0;
        }
    }
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
    let xy: Vec<(f64, f64)> = build_trajectory(&path, msg)
        .into_iter()
        .map(|p| (p.get_location().x(), p.get_location().y()))
        .collect();
    let poses = smooth_path(&xy, &grid, msg)
        .into_iter()
        .map(|(x, y)| loc::Pose::new(x, y, 0.0))
        .collect();
    Some(snap_start_to_pose(poses, pose, msg.resolution))
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

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> Costmap {
        Costmap {
            rows: 1,
            cols: 2,
            resolution: 0.05,
            origin_x: -25.0,
            origin_y: -25.0,
            data: vec![],
        }
    }

    #[test]
    fn straight_plan_starts_on_the_pose() {
        // floor((0 - (-25)) / 0.05) is 499. A corner vertex there is 5 cm off.
        let path = build_trajectory(&[(499, 499), (499, 519)], &map());
        let pose = loc::Pose::new(0.0, 0.0, 0.0);
        let snapped = snap_start_to_pose(path, &pose, 0.05);
        let start = snapped[0].get_location();
        let ahead = snapped[1].get_location();
        assert!(start.x().abs() < 1e-9 && start.y().abs() < 1e-9);
        assert!((ahead.x() - 1.0).abs() < 1e-9, "x={}", ahead.x());
        assert!(ahead.y().abs() < 1e-9, "y={}", ahead.y());
    }

    #[test]
    fn snapped_start_stays_put_when_the_cell_is_far() {
        let path = build_trajectory(&[(499, 480)], &map());
        let pose = loc::Pose::new(0.0, 0.0, 0.0);
        let snapped = snap_start_to_pose(path, &pose, 0.05);
        assert!(snapped[0].get_location().x().abs() > 0.5);
    }

    fn blank(rows: usize, cols: usize, origin_x: f64, origin_y: f64) -> (Grid, Costmap) {
        let data = vec![0u8; rows * cols];
        (
            Grid {
                rows,
                cols,
                data: data.clone(),
            },
            Costmap {
                rows,
                cols,
                resolution: 0.05,
                origin_x,
                origin_y,
                data,
            },
        )
    }

    fn fill_rect(grid: &mut Grid, map: &Costmap, x0: f64, y0: f64, x1: f64, y1: f64) {
        let res = map.resolution;
        let c0 = ((x0.min(x1) - map.origin_x) / res).floor().max(0.0) as isize;
        let r0 = ((y0.min(y1) - map.origin_y) / res).floor().max(0.0) as isize;
        let c1 = ((x0.max(x1) - map.origin_x) / res)
            .ceil()
            .min(grid.cols as f64) as isize;
        let r1 = ((y0.max(y1) - map.origin_y) / res)
            .ceil()
            .min(grid.rows as f64) as isize;
        for r in r0..r1 {
            for c in c0..c1 {
                grid.data[r as usize * grid.cols + c as usize] = 255;
            }
        }
    }

    fn menger(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        let ab = dist(a, b);
        let bc = dist(b, c);
        let ca = dist(c, a);
        let cross = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
        if ab < 1e-9 || bc < 1e-9 || ca < 1e-9 {
            return 0.0;
        }
        cross.abs() * 2.0 / (ab * bc * ca)
    }

    #[test]
    fn open_l_stays_within_turn_radius() {
        let (_, map) = blank(220, 220, 0.0, 0.0);
        let pose = loc::Pose::new(1.025, 1.025, 0.0);
        let goal = Goal {
            latitude: 9.025,
            longitude: 9.025,
        };
        let traj = plan_once(&map, &pose, goal).expect("path");
        let xy: Vec<(f64, f64)> = traj
            .iter()
            .map(|p| (p.get_location().x(), p.get_location().y()))
            .collect();
        assert_eq!(
            xy.len(),
            2,
            "open space shortcuts to a straight line, got {xy:?}"
        );
        let end = xy.last().unwrap();
        assert!((end.0 - 9.025).abs() < 1e-6 && (end.1 - 9.025).abs() < 1e-6);
    }

    #[test]
    fn corner_around_a_block_stays_off_it_and_within_radius() {
        let (mut grid, map) = blank(400, 400, -5.0, -5.0);
        // Interior of the L. The unshifted arc bulges into this; the outside stays free.
        fill_rect(&mut grid, &map, 1.0, 1.7, 8.4, 9.0);
        let l = [(1.0, 1.0), (9.0, 1.0), (9.0, 9.0)];
        let smoothed = smooth_path(&l, &grid, &map);
        assert!(
            smoothed.len() > 3,
            "corner should become an arc, got {smoothed:?}"
        );
        let limit = 1.0 / min_turn_radius();
        for w in smoothed.windows(3) {
            let k = menger(w[0], w[1], w[2]);
            assert!(k <= limit + 1e-8, "curvature {k} exceeds {limit}");
        }
        for w in smoothed.windows(2) {
            assert!(
                segment_clear(w[0], w[1], &grid, &map),
                "arc entered the block"
            );
        }
        let corner = (9.0, 1.0);
        let nearest = smoothed
            .iter()
            .map(|&p| dist(p, corner))
            .fold(f64::MAX, f64::min);
        assert!(nearest > 0.5, "sharp corner survived, nearest={nearest}");
    }

    #[test]
    fn straight_path_is_unchanged() {
        let (grid, map) = blank(100, 100, 0.0, 0.0);
        let line = [(0.5, 1.0), (1.5, 1.0), (2.5, 1.0), (3.5, 1.0)];
        let smoothed = smooth_path(&line, &grid, &map);
        assert_eq!(smoothed, vec![line[0], *line.last().unwrap()]);
        assert_eq!(
            smooth_path(&[line[0], line[3]], &grid, &map),
            vec![line[0], line[3]]
        );
    }
}
