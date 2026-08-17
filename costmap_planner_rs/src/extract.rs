use nav2_msgs::msg::Costmap;

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
    let cols = msg.metadata.size_x as usize;
    let rows = msg.metadata.size_y as usize;

    Grid {
        rows,
        cols,
        data: msg.data.clone(),
    }
}
