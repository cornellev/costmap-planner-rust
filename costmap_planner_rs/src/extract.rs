use std::collections::VecDeque;

pub struct Costmap {
    pub rows: usize,
    pub cols: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>,
}

pub fn decode_costmap(payload: &[u8]) -> Result<Costmap, &'static str> {
    let (header, data) = payload
        .split_first_chunk::<32>()
        .ok_or("costmap payload is shorter than its 32-byte header")?;

    // Timestamps (bytes 0..8) and the reserved field (bytes 28..32) are unused.
    // just generally taken from sophia's occupancy grid
    Ok(Costmap {
        cols: u32::from_le_bytes([header[8], header[9], header[10], header[11]]) as usize,
        rows: u32::from_le_bytes([header[12], header[13], header[14], header[15]]) as usize,
        resolution: f32::from_le_bytes([header[16], header[17], header[18], header[19]]) as f64,
        origin_x: f32::from_le_bytes([header[20], header[21], header[22], header[23]]) as f64,
        origin_y: f32::from_le_bytes([header[24], header[25], header[26], header[27]]) as f64,
        data: data.to_vec(),
    })
}

pub struct Grid {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<u8>,
}

impl Grid {
    pub fn at(&self, r: usize, c: usize) -> u8 {
        self.data[r * self.cols + c]
    }
}

pub fn costmap_to_grid(msg: &Costmap) -> Grid {
    Grid {
        rows: msg.rows,
        cols: msg.cols,
        data: msg.data.clone(),
    }
}

/// Widen obstacles before search. 8-connected hops are Chebyshev distance, so the
/// lethal region is an axis-aligned square of half-width `inscribed_m`.
pub fn inflate(grid: &mut Grid, resolution_m: f64, inscribed_m: f64, decay_m: f64) {
    if grid.rows == 0 || grid.cols == 0 || !(resolution_m > 0.0) || !resolution_m.is_finite() {
        return;
    }
    let reach_m = inscribed_m + decay_m.max(0.0);
    let mut dist = vec![u32::MAX; grid.rows * grid.cols];
    let mut queue = VecDeque::new();
    for (idx, cost) in grid.data.iter().enumerate() {
        if *cost >= 253 {
            dist[idx] = 0;
            queue.push_back(idx);
        }
    }
    // surroundings
    let steps = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1),
    ];
    while let Some(idx) = queue.pop_front() {
        let r = (idx / grid.cols) as isize;
        let c = (idx % grid.cols) as isize;
        let next = dist[idx] + 1;
        if (next as f64) * resolution_m > reach_m {
            continue;
        }
        for (dr, dc) in steps {
            let nr = r + dr;
            let nc = c + dc;
            if nr < 0 || nc < 0 || nr >= grid.rows as isize || nc >= grid.cols as isize {
                continue;
            }
            let nidx = nr as usize * grid.cols + nc as usize;
            if dist[nidx] != u32::MAX {
                continue;
            }
            dist[nidx] = next;
            queue.push_back(nidx);
        }
    }
    for (idx, cell) in grid.data.iter_mut().enumerate() {
        if dist[idx] == u32::MAX {
            continue;
        }
        let dist_m = dist[idx] as f64 * resolution_m;
        if dist_m <= inscribed_m {
            *cell = 255;
        } else if decay_m > 0.0 && dist_m <= reach_m {
            // just an easy linear decay, not a gaussian or anything fancy
            let t = (dist_m - inscribed_m) / decay_m;
            *cell = (252.0 * (1.0 - t)).round().clamp(1.0, 252.0) as u8;
        }
    }
}

// just some tests that i had chat generate to check the inflation function, not really meant to be exhaustive
#[cfg(test)]
mod tests {
    use super::*;

    fn obstacle_grid(rows: usize, cols: usize, r: usize, c: usize) -> Grid {
        let mut data = vec![0u8; rows * cols];
        data[r * cols + c] = 255;
        Grid { rows, cols, data }
    }

    #[test]
    fn lethal_square_covers_half_width_and_decay_stays_drivable() {
        let mut grid = obstacle_grid(41, 41, 20, 20);
        inflate(&mut grid, 0.05, 0.31, 0.5);
        assert_eq!(grid.at(20, 26), 255, "0.30 m is inside the 0.31 m square");
        let decayed = grid.at(20, 27);
        assert!(decayed > 0 && decayed < 253, "0.35 m is decay, got {decayed}");
        assert_eq!(grid.at(20, 40), 0, "1.00 m is past the decay band");
    }

    #[test]
    fn cone_offset_blocks_the_centerline() {
        let mut grid = obstacle_grid(11, 11, 5, 5);
        inflate(&mut grid, 0.05, 0.31, 0.5);
        // 3 cells * 0.05 m = 0.15 m, the cell at the cone's 0.14 m offset.
        assert_eq!(grid.at(5, 8), 255);
    }
}
