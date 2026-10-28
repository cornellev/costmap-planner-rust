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
