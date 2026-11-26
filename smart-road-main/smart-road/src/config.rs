//! All physical quantities use metres, seconds and metres/second.
pub const DT: f64 = 1.0 / 120.0;
pub const WORLD_HALF: f64 = 60.0;
pub const LANE_WIDTH: f64 = 6.0;
pub const ROAD_HALF: f64 = 3.0 * LANE_WIDTH;
pub const CAR_LENGTH: f64 = 4.0;
pub const CAR_WIDTH: f64 = 2.0;
// sqrt((length/2)^2 + (width/2)^2): encloses the rotated sprite.
pub const BODY_RADIUS: f64 = 2.236_067_977_499_79;
pub const SAFETY_DISTANCE: f64 = 1.0;
pub const REQUIRED_CENTRE_DISTANCE: f64 = 2.0 * BODY_RADIUS + SAFETY_DISTANCE;
// Bigger than REQUIRED_CENTRE_DISTANCE, including curve chord shortening.
pub const FOLLOW_SPACING: f64 = 8.0;
// Keeps waiting cars and released cars clear of the physical junction.
pub const CONTROL_PADDING: f64 = 6.5;
pub const REQUEST_DISTANCE: f64 = 10.0;
pub const FAST_SPEED: f64 = 12.0;
pub const STRAIGHT_SPEED: f64 = 10.0;
pub const LEFT_SPEED: f64 = 6.0;
pub const RIGHT_SPEED: f64 = 4.0;
pub const ACCELERATION: f64 = 3.5;
pub const BRAKING: f64 = 6.0;
pub const POSITION_MARGIN: f64 = 0.05;
// Moderate demonstration load; decrease the interval for saturation tests.
pub const AUTO_INTERVAL: f64 = 2.5;
pub const APPROACH_SPAWN_INTERVAL: f64 = 0.25;
pub const MAX_PENDING_PER_ROUTE: usize = 64;
pub const ROUTE_COUNT: usize = 12;
// Polyline approximation of arcs. Maximum two-curve sagitta error is
// <= 2 * SAMPLE_STEP^2 / (8 * MIN_TURN_RADIUS) = 0.02084 m.
pub const SAMPLE_STEP: f64 = 0.5;
pub const CONFLICT_MARGIN: f64 = 0.10;
pub const MIN_TURN_RADIUS: f64 = 3.0;
