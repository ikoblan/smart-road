use crate::geometry::{Path, Vec2};

#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: u64,
    /// Immutable index of its lane/route. Never changed after creation.
    pub route: usize,
    /// Arc length travelled since actual vehicle creation, in metres.
    pub distance: f64,
    pub velocity: f64,
    pub detected_at: f64,
    pub time: f64,
    pub admitted: bool,
    pub requested_at: Option<u64>,
    pub waiting_time: f64,
    pub sprite: usize,
}
impl Vehicle {
    pub fn position(&self, paths: &[Path]) -> Vec2 { paths[self.route].pose(self.distance).0 }
    pub fn distance_to_exit(&self, paths: &[Path]) -> f64 { (paths[self.route].length - self.distance).max(0.0) }
    pub fn has_reservation(&self, paths: &[Path]) -> bool { self.admitted && self.distance < paths[self.route].release_s }
}
