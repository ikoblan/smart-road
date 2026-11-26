use crate::config::*;

pub fn velocity(distance: f64, time: f64) -> Result<f64, &'static str> {
    if !distance.is_finite() || distance < 0.0 { return Err("distance must be finite and nonnegative"); }
    if !time.is_finite() || time <= 0.0 { return Err("time must be finite and strictly positive"); }
    Ok(distance / time)
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Proximity { Clear, Stop, Collision }
/// Gap between conservative vehicle envelopes, not centre-to-centre distance.
pub fn proximity(surface_gap: f64) -> Proximity {
    if !surface_gap.is_finite() || surface_gap < 0.0 { Proximity::Collision }
    else if surface_gap <= SAFETY_DISTANCE { Proximity::Stop }
    else { Proximity::Clear }
}
/// Speed whose next step plus full emergency stopping distance fits in room.
/// v*dt + v^2/(2*b) <= room. This assumes a stationary obstacle ahead.
pub fn stopping_speed(room: f64) -> f64 {
    if room <= 0.0 { return 0.0; }
    BRAKING * ((DT * DT + 2.0 * room / BRAKING).sqrt() - DT)
}
/// Conservative following; returns zero at or below the safety boundary.
pub fn following_speed(surface_gap: f64) -> f64 {
    if proximity(surface_gap) != Proximity::Clear { 0.0 }
    else { stopping_speed((surface_gap - SAFETY_DISTANCE - POSITION_MARGIN).max(0.0)) }
}
#[derive(Clone, Copy, Debug)]
pub struct Motion { pub distance: f64, pub speed: f64, pub hard_brake: bool }
/// Semi-implicit fixed-step integration: ds = v_next * DT. Normal motion has
/// bounded acceleration/braking. A hard position limit is a final fail-safe;
/// it is counted rather than concealed as ordinary bounded deceleration.
pub fn advance(speed: f64, target: f64, room: f64) -> Motion {
    let target = target.clamp(0.0, FAST_SPEED);
    let next = if target > speed { (speed + ACCELERATION * DT).min(target) }
        else { (speed - BRAKING * DT).max(target) };
    let distance = (next * DT).min(room.max(0.0));
    let actual = distance / DT;
    Motion { distance, speed: actual, hard_brake: actual + BRAKING * DT + 1e-8 < speed }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn distance_over_time_at_several_speeds() {
        for v in [0.0, 4.0, 6.0, 10.0, 12.0] { for t in [0.1, 1.0, 17.5] { assert!((velocity(v * t, t).unwrap() - v).abs() < 1e-10); } }
    }
    #[test]
    fn invalid_physics_inputs_are_rejected() {
        for (d, t) in [(1.0, 0.0), (1.0, -1.0), (-2.0, 1.0), (f64::NAN, 1.0), (1.0, f64::INFINITY)] { assert!(velocity(d, t).is_err()); }
    }
    #[test]
    fn safety_boundary_is_inclusive() {
        assert_eq!(proximity(SAFETY_DISTANCE), Proximity::Stop);
        assert_eq!(proximity(SAFETY_DISTANCE - 0.001), Proximity::Stop);
        assert_eq!(proximity(SAFETY_DISTANCE + 0.001), Proximity::Clear);
        assert_eq!(following_speed(SAFETY_DISTANCE), 0.0);
        assert_eq!(proximity(-0.001), Proximity::Collision);
    }
    #[test]
    fn stopping_speed_respects_stopping_inequality() {
        for room in [0.0, 0.01, 0.1, 1.0, 10.0, 100.0] {
            let v = stopping_speed(room);
            assert!(v * DT + v * v / (2.0 * BRAKING) <= room + 1e-9);
        }
    }
    #[test]
    fn integrator_obeys_velocity_equals_distance_over_time() {
        for speed in [0.0, 4.0, 10.0, 12.0] {
            let m = advance(speed, 6.0, 100.0);
            assert!((velocity(m.distance, DT).unwrap() - m.speed).abs() < 1e-10);
        }
    }
    #[test]
    fn acceleration_and_braking_are_bounded_without_override() {
        assert!((advance(0.0, FAST_SPEED, 100.0).speed - ACCELERATION * DT).abs() < 1e-10);
        assert!((advance(FAST_SPEED, 0.0, 100.0).speed - (FAST_SPEED - BRAKING * DT)).abs() < 1e-10);
        let m = advance(FAST_SPEED, 0.0, 0.001);
        assert!(m.hard_brake);
        assert!(m.distance <= 0.001);
    }
}
