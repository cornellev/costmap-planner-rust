use serde::Deserialize;

// i have no clue what way the data is stored, so this is a guess
// lowk this should be an occupancy grid but i'll adjust that as we go or just do fake conversion middlenode lol
#[derive(Deserialize)]
pub struct Costmap {
    pub rows: usize,
    pub cols: usize,
    pub resolution: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>,
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
