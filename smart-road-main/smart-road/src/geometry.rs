use crate::config::*;
use std::f64::consts::{FRAC_PI_2, PI};
use std::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}
impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self { Self { x, y } }
    pub fn dot(self, rhs: Self) -> f64 { self.x * rhs.x + self.y * rhs.y }
    pub fn length(self) -> f64 { self.dot(self).sqrt() }
    pub fn distance(self, rhs: Self) -> f64 { (self - rhs).length() }
    pub fn cross(self, rhs: Self) -> f64 { self.x * rhs.y - self.y * rhs.x }
    /// Quarter turns in screen coordinates: positive y points down.
    pub fn rotate_quarters(self, quarters: usize) -> Self {
        match quarters % 4 {
            0 => self,
            1 => Self::new(-self.y, self.x),
            2 => Self::new(-self.x, -self.y),
            _ => Self::new(self.y, -self.x),
        }
    }
}
impl Add for Vec2 { type Output = Self; fn add(self, rhs: Self) -> Self { Self::new(self.x + rhs.x, self.y + rhs.y) } }
impl Sub for Vec2 { type Output = Self; fn sub(self, rhs: Self) -> Self { Self::new(self.x - rhs.x, self.y - rhs.y) } }
impl Mul<f64> for Vec2 { type Output = Self; fn mul(self, rhs: f64) -> Self { Self::new(self.x * rhs, self.y * rhs) } }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum Approach { North = 0, East = 1, South = 2, West = 3 }
impl Approach {
    pub const ALL: [Self; 4] = [Self::North, Self::East, Self::South, Self::West];
    pub fn name(self) -> &'static str {
        match self { Self::North => "NORTH", Self::East => "EAST", Self::South => "SOUTH", Self::West => "WEST" }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum Turn { Right = 0, Straight = 1, Left = 2 }
impl Turn {
    pub const ALL: [Self; 3] = [Self::Right, Self::Straight, Self::Left];
    pub fn name(self) -> &'static str {
        match self { Self::Right => "RIGHT", Self::Straight => "STRAIGHT", Self::Left => "LEFT" }
    }
    pub fn speed(self) -> f64 {
        match self { Self::Right => RIGHT_SPEED, Self::Straight => STRAIGHT_SPEED, Self::Left => LEFT_SPEED }
    }
}

#[derive(Clone, Debug)]
pub struct Path {
    pub approach: Approach,
    pub turn: Turn,
    pub entry_s: f64,
    pub gate_s: f64,
    pub exit_s: f64,
    pub release_s: f64,
    pub length: f64,
    pub inner_length: f64,
    offset: f64,
    radius: f64,
}
impl Path {
    pub fn new(approach: Approach, turn: Turn) -> Self {
        let offset = match turn { Turn::Right => 2.5 * LANE_WIDTH, Turn::Straight => 1.5 * LANE_WIDTH, Turn::Left => 0.5 * LANE_WIDTH };
        let radius = match turn { Turn::Right => ROAD_HALF - offset, Turn::Left => ROAD_HALF + offset, Turn::Straight => 0.0 };
        let inner_length = if turn == Turn::Straight { 2.0 * ROAD_HALF } else { radius * FRAC_PI_2 };
        let entry_s = WORLD_HALF + BODY_RADIUS - ROAD_HALF;
        let exit_s = entry_s + inner_length;
        Self { approach, turn, offset, radius, inner_length, entry_s,
            gate_s: entry_s - CONTROL_PADDING, exit_s,
            release_s: exit_s + CONTROL_PADDING, length: exit_s + entry_s }
    }
    pub fn id(&self) -> usize { self.approach as usize * 3 + self.turn as usize }
    pub fn label(&self) -> String { format!("{} / {}", self.approach.name(), self.turn.name()) }
    /// Analytic arc-length parameterization: a move of ds travels exactly ds.
    /// Returns (centre, unit tangent); the sprite heading is atan2(t.y,t.x).
    pub fn pose(&self, distance: f64) -> (Vec2, Vec2) {
        let s = distance.clamp(0.0, self.length);
        let (p, t) = if s <= self.entry_s {
            (Vec2::new(-self.offset, -WORLD_HALF - BODY_RADIUS + s), Vec2::new(0.0, 1.0))
        } else if s < self.exit_s {
            let u = s - self.entry_s;
            match self.turn {
                Turn::Straight => (Vec2::new(-self.offset, -ROAD_HALF + u), Vec2::new(0.0, 1.0)),
                Turn::Right => {
                    let a = u / self.radius;
                    (Vec2::new(-ROAD_HALF + self.radius * a.cos(), -ROAD_HALF + self.radius * a.sin()), Vec2::new(-a.sin(), a.cos()))
                }
                Turn::Left => {
                    let a = PI - u / self.radius;
                    (Vec2::new(ROAD_HALF + self.radius * a.cos(), -ROAD_HALF + self.radius * a.sin()), Vec2::new(a.sin(), -a.cos()))
                }
            }
        } else {
            let u = s - self.exit_s;
            match self.turn {
                Turn::Straight => (Vec2::new(-self.offset, ROAD_HALF + u), Vec2::new(0.0, 1.0)),
                Turn::Right => (Vec2::new(-ROAD_HALF - u, -self.offset), Vec2::new(-1.0, 0.0)),
                Turn::Left => (Vec2::new(ROAD_HALF + u, self.offset), Vec2::new(1.0, 0.0)),
            }
        };
        (p.rotate_quarters(self.approach as usize), t.rotate_quarters(self.approach as usize))
    }
    pub fn control_polyline(&self) -> Vec<Vec2> {
        let mut result = Vec::new();
        // Include the line/arc junctions explicitly, not just uniform samples.
        let bounds = [self.gate_s, self.entry_s, self.exit_s, self.release_s];
        result.push(self.pose(bounds[0]).0);
        for pair in bounds.windows(2) {
            let n = ((pair[1] - pair[0]) / SAMPLE_STEP).ceil() as usize;
            for k in 1..=n {
                let s = pair[0] + (pair[1] - pair[0]) * k as f64 / n as f64;
                result.push(self.pose(s).0);
            }
        }
        result
    }
}
pub fn all_paths() -> Vec<Path> {
    Approach::ALL.into_iter().flat_map(|a| Turn::ALL.into_iter().map(move |t| Path::new(a, t))).collect()
}

pub fn point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let denom = d.dot(d);
    if denom <= 1e-20 { return p.distance(a); }
    let t = ((p - a).dot(d) / denom).clamp(0.0, 1.0);
    p.distance(a + d * t)
}
pub fn segment_distance(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> f64 {
    let ab = b - a;
    let cd = d - c;
    let denom = ab.cross(cd);
    if denom.abs() > 1e-12 {
        let u = (c - a).cross(cd) / denom;
        let v = (c - a).cross(ab) / denom;
        if (0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v) { return 0.0; }
    }
    point_segment_distance(a, c, d).min(point_segment_distance(b, c, d))
        .min(point_segment_distance(c, a, b)).min(point_segment_distance(d, a, b))
}
/// Minimum centre separation during simultaneous, linearly interpolated moves.
/// Used with an additional arc-sagitta bound by the safety monitor.
pub fn swept_centre_distance(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> f64 {
    let relative_start = a0 - b0;
    let relative_move = (a1 - a0) - (b1 - b0);
    let denom = relative_move.dot(relative_move);
    let t = if denom > 1e-20 { (-relative_start.dot(relative_move) / denom).clamp(0.0, 1.0) } else { 0.0 };
    (relative_start + relative_move * t).length()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn twelve_distinct_lane_routes() {
        let paths = all_paths();
        assert_eq!(paths.len(), 12);
        for (i, p) in paths.iter().enumerate() { assert_eq!(p.id(), i); }
    }
    #[test]
    fn curves_join_continuously_and_headings_are_unit_vectors() {
        for p in all_paths() {
            for s in [p.entry_s, p.exit_s] {
                let (a, ta) = p.pose(s - 1e-6);
                let (b, tb) = p.pose(s + 1e-6);
                assert!(a.distance(b) < 3e-6);
                assert!(ta.distance(tb) < 2e-6);
            }
            for k in 0..=100 { assert!((p.pose(p.length * k as f64 / 100.0).1.length() - 1.0).abs() < 1e-10); }
        }
    }
    #[test]
    fn north_turns_have_correct_exit_directions() {
        assert_eq!(Path::new(Approach::North, Turn::Right).pose(1e6).1, Vec2::new(-1.0, 0.0));
        assert_eq!(Path::new(Approach::North, Turn::Left).pose(1e6).1, Vec2::new(1.0, 0.0));
    }
    #[test]
    fn segment_crossing_and_parallel_distances() {
        assert_eq!(segment_distance(Vec2::new(-1.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, -1.0), Vec2::new(0.0, 1.0)), 0.0);
        assert!((segment_distance(Vec2::new(0.0, 0.0), Vec2::new(4.0, 0.0), Vec2::new(1.0, 3.0), Vec2::new(5.0, 3.0)) - 3.0).abs() < 1e-9);
    }
    #[test]
    fn swept_monitor_detects_tunnelling() {
        assert!(swept_centre_distance(Vec2::new(-10.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(0.0, -10.0), Vec2::new(0.0, 10.0)) < 1e-9);
    }
}
