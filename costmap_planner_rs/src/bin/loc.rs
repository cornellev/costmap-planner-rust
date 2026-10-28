use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Location {
    x: f64,
    y: f64
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Pose {
    location: Location,
    heading: f64
}

impl Location {
    pub fn new(x: f64, y: f64) -> Location {
        Location { x, y }
    }

    pub fn distance_to(&self, other: &Location) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn equals(&self, other: &Location) -> bool {
        self.x == other.x && self.y == other.y
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }
}

impl Pose {
    pub fn new(x: f64, y: f64, heading: f64) -> Pose {
        Pose { location: Location::new(x, y), heading }
    }

    pub fn distance_to(&self, other: &Pose) -> f64 {
        self.location.distance_to(&other.location)
    }

    pub fn equals(&self, other: &Pose) -> bool {
        self.location.x == other.location.x && self.location.y == other.location.y && self.heading == other.heading
    }

    pub fn atan2(&self) -> f64 {
        self.location.y.atan2(self.location.x)
    }

    pub fn get_heading(&self) -> f64 {
        self.heading
    }

    pub fn get_location(&self) -> Location {
        self.location
    }
}