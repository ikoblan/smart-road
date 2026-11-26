use std::collections::HashSet;
use crate::config::*;
use crate::controller::ConflictMatrix;
use crate::geometry::{all_paths, swept_centre_distance, Approach, Path, Turn, Vec2};
use crate::physics::{advance, stopping_speed};
use crate::stats::Statistics;
use crate::vehicle::Vehicle;

#[derive(Clone, Debug)]
struct Rng(u64);
impl Rng {
    /// SplitMix64. Seeded, reproducible, and sufficient for traffic arrivals.
    fn index(&mut self, bound: usize) -> usize {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) % bound as u64) as usize
    }
}

#[derive(Clone, Debug)]
pub struct Simulation {
    pub paths: Vec<Path>,
    pub conflicts: ConflictMatrix,
    pub vehicles: Vec<Vehicle>,
    pub stats: Statistics,
    pub tick: u64,
    pub auto: bool,
    pending: [usize; ROUTE_COUNT],
    spawn_cursor: [usize; 4],
    next_spawn_tick: [u64; 4],
    next_auto_tick: u64,
    next_id: u64,
    next_ticket: u64,
    close_pairs: HashSet<(u64, u64)>,
    collision_pairs: HashSet<(u64, u64)>,
    rng: Rng,
}
impl Simulation {
    pub fn new(seed: u64) -> Self {
        let paths = all_paths();
        let conflicts = ConflictMatrix::build(&paths);
        Self::with_layout(seed, paths, conflicts)
    }
    pub fn with_layout(seed: u64, paths: Vec<Path>, conflicts: ConflictMatrix) -> Self {
        assert_eq!(paths.len(), ROUTE_COUNT);
        Self { paths, conflicts, vehicles: Vec::new(), stats: Statistics::default(),
            tick: 0, auto: false, pending: [0; ROUTE_COUNT], spawn_cursor: [0; 4],
            next_spawn_tick: [0; 4], next_auto_tick: 0, next_id: 1, next_ticket: 1,
            close_pairs: HashSet::new(), collision_pairs: HashSet::new(), rng: Rng(seed) }
    }
    pub fn time(&self) -> f64 { self.tick as f64 * DT }
    pub fn pending_count(&self) -> usize { self.pending.iter().sum() }
    pub fn pending_on(&self, route: usize) -> usize { self.pending[route] }
    pub fn active_reservations(&self) -> usize {
        self.vehicles.iter().filter(|v| v.has_reservation(&self.paths)).count()
    }
    pub fn set_auto(&mut self, enabled: bool) {
        if enabled && !self.auto { self.next_auto_tick = self.tick; }
        self.auto = enabled;
    }
    pub fn request(&mut self, approach: Approach, turn: Turn) -> bool {
        let route = approach as usize * 3 + turn as usize;
        if self.pending[route] >= MAX_PENDING_PER_ROUTE {
            self.stats.rejected_requests += 1;
            return false;
        }
        self.pending[route] += 1;
        self.stats.peak_pending = self.stats.peak_pending.max(self.pending_count());
        true
    }
    pub fn request_random_from(&mut self, approach: Approach) {
        let turn = Turn::ALL[self.rng.index(3)];
        self.request(approach, turn);
    }
    pub fn request_random(&mut self) {
        let approach = Approach::ALL[self.rng.index(4)];
        self.request_random_from(approach);
    }
    fn spawn_available(&mut self) {
        let now = self.time();
        for approach in 0..4 {
            if self.tick < self.next_spawn_tick[approach] { continue; }
            for offset in 0..3 {
                let turn = (self.spawn_cursor[approach] + offset) % 3;
                let route = approach * 3 + turn;
                if self.pending[route] == 0 { continue; }
                let nearest = self.vehicles.iter().filter(|v| v.route == route)
                    .map(|v| v.distance).fold(f64::INFINITY, f64::min);
                if nearest < FOLLOW_SPACING + 2.0 * POSITION_MARGIN { continue; }
                let initial_speed = if nearest.is_finite() {
                    FAST_SPEED.min(stopping_speed(nearest - FOLLOW_SPACING - POSITION_MARGIN))
                } else { FAST_SPEED };
                self.vehicles.push(Vehicle { id: self.next_id, route, distance: 0.0,
                    velocity: initial_speed, detected_at: now, time: 0.0, admitted: false,
                    requested_at: None, waiting_time: 0.0, sprite: (self.next_id % 4) as usize });
                self.next_id += 1;
                self.pending[route] -= 1;
                self.stats.spawned += 1;
                self.stats.observe_velocity(initial_speed);
                self.spawn_cursor[approach] = (turn + 1) % 3;
                self.next_spawn_tick[approach] = self.tick + (APPROACH_SPAWN_INTERVAL / DT).ceil() as u64;
                break;
            }
        }
    }
    fn schedule(&mut self) {
        let mut heads: [Option<usize>; ROUTE_COUNT] = [None; ROUTE_COUNT];
        for (i, v) in self.vehicles.iter().enumerate() {
            if v.admitted { continue; }
            if heads[v.route].map_or(true, |j| v.distance > self.vehicles[j].distance) { heads[v.route] = Some(i); }
        }
        let mut candidates: Vec<usize> = heads.into_iter().flatten().filter(|&i| {
            let v = &self.vehicles[i];
            v.distance >= self.paths[v.route].gate_s - REQUEST_DISTANCE
        }).collect();
        candidates.sort_by_key(|&i| self.vehicles[i].id);
        for &i in &candidates {
            if self.vehicles[i].requested_at.is_none() {
                self.vehicles[i].requested_at = Some(self.next_ticket);
                self.next_ticket += 1;
            }
        }
        candidates.sort_by_key(|&i| self.vehicles[i].requested_at);
        let requests: Vec<_> = candidates.iter().map(|&i| (self.vehicles[i].id, self.vehicles[i].route)).collect();
        let active: Vec<_> = self.vehicles.iter().filter(|v| v.has_reservation(&self.paths)).map(|v| v.route).collect();
        let admitted = self.conflicts.admit(&requests, &active);
        for v in &mut self.vehicles { if admitted.contains(&v.id) { v.admitted = true; } }
    }
    /// One authoritative 1/120-second tick; rendering never changes the state.
    pub fn step(&mut self) {
        if self.auto && self.tick >= self.next_auto_tick {
            self.request_random();
            self.next_auto_tick = self.tick + (AUTO_INTERVAL / DT).round() as u64;
        }
        self.spawn_available();
        self.schedule();
        self.stats.peak_active = self.stats.peak_active.max(self.vehicles.len());
        self.stats.peak_reservations = self.stats.peak_reservations.max(self.active_reservations());

        // All decisions read the same snapshot, avoiding iteration-order races.
        let snapshot = self.vehicles.clone();
        let old_positions: Vec<_> = snapshot.iter().map(|v| v.position(&self.paths)).collect();
        let next_time = self.time() + DT;
        let mut moves = Vec::with_capacity(snapshot.len());
        for v in &snapshot {
            let path = &self.paths[v.route];
            let mut target = if v.distance >= path.exit_s { FAST_SPEED } else {
                // Reach the appropriate cornering speed by the start of the arc.
                let to_entry = (path.entry_s - v.distance - POSITION_MARGIN).max(0.0);
                (path.turn.speed().powi(2) + 2.0 * BRAKING * to_entry).sqrt().min(FAST_SPEED)
            };
            let mut room = f64::INFINITY;
            if !v.admitted { room = (path.gate_s - POSITION_MARGIN - v.distance).max(0.0); }
            for leader in &snapshot {
                if leader.route == v.route && leader.distance > v.distance {
                    room = room.min((leader.distance - v.distance - FOLLOW_SPACING - POSITION_MARGIN).max(0.0));
                }
            }
            if room.is_finite() { target = target.min(stopping_speed((room - POSITION_MARGIN).max(0.0))); }
            moves.push(advance(v.velocity, target, room));
        }
        for (i, v) in self.vehicles.iter_mut().enumerate() {
            let movement = moves[i];
            let remaining = self.paths[v.route].length - v.distance;
            v.distance += movement.distance;
            v.velocity = movement.speed;
            if movement.distance > 0.0 && remaining <= movement.distance {
                // Interpolate the exact removal instant within the final tick.
                v.time = next_time - DT + (remaining / movement.distance) * DT - v.detected_at;
            } else { v.time = next_time - v.detected_at; }
            if v.velocity <= 0.05 { v.waiting_time += DT; }
            if movement.hard_brake { self.stats.hard_brakes += 1; }
            self.stats.observe_velocity(v.velocity);
        }
        self.monitor_safety(&old_positions, &moves.iter().map(|m| m.distance).collect::<Vec<_>>());
        let mut finished = Vec::new();
        self.vehicles.retain(|v| {
            if v.distance >= self.paths[v.route].length {
                finished.push((v.time, self.paths[v.route].length, v.waiting_time));
                false
            } else { true }
        });
        for (time, distance, waiting) in finished { self.stats.record_pass(time, distance, waiting); }
        self.tick += 1;
    }
    fn monitor_safety(&mut self, old: &[Vec2], distances: &[f64]) {
        let mut close_now = HashSet::new();
        let mut collisions_now = HashSet::new();
        for i in 0..self.vehicles.len() {
            for j in i + 1..self.vehicles.len() {
                let a = &self.vehicles[i];
                let b = &self.vehicles[j];
                let linear_distance = swept_centre_distance(old[i], a.position(&self.paths), old[j], b.position(&self.paths));
                // Conservative bound for the deviation of two moving arcs from
                // their chords during this tick. Straight paths also use it.
                let curve_error = (distances[i].powi(2) + distances[j].powi(2)) / (8.0 * MIN_TURN_RADIUS);
                let gap = linear_distance - 2.0 * BODY_RADIUS - curve_error;
                self.stats.observe_gap(gap);
                let pair = (a.id.min(b.id), a.id.max(b.id));
                if gap <= SAFETY_DISTANCE {
                    close_now.insert(pair);
                    if !self.close_pairs.contains(&pair) { self.stats.close_calls += 1; }
                }
                if gap < 0.0 {
                    collisions_now.insert(pair);
                    if !self.collision_pairs.contains(&pair) { self.stats.collisions += 1; }
                }
            }
        }
        self.close_pairs = close_now;
        self.collision_pairs = collisions_now;
    }
    pub fn minimum_current_gap(&self) -> Option<f64> {
        let mut minimum: Option<f64> = None;
        for (i, a) in self.vehicles.iter().enumerate() {
            for b in &self.vehicles[i + 1..] {
                let gap = a.position(&self.paths).distance(b.position(&self.paths)) - 2.0 * BODY_RADIUS;
                minimum = Some(minimum.map_or(gap, |m| m.min(gap)));
            }
        }
        minimum
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn repeat_requests_are_queued_not_overlapped() {
        let mut sim = Simulation::new(1);
        for _ in 0..10 { assert!(sim.request(Approach::South, Turn::Straight)); }
        sim.step();
        assert_eq!(sim.vehicles.len(), 1);
        assert_eq!(sim.pending_count(), 9);
        for _ in 0..600 {
            sim.step();
            if let Some(gap) = sim.minimum_current_gap() { assert!(gap > SAFETY_DISTANCE); }
        }
        assert_eq!(sim.stats.collisions, 0);
    }
    #[test]
    fn spam_queue_is_bounded_and_rejections_are_visible() {
        let mut sim = Simulation::new(1);
        for _ in 0..MAX_PENDING_PER_ROUTE + 10 { sim.request(Approach::North, Turn::Left); }
        assert_eq!(sim.pending_count(), MAX_PENDING_PER_ROUTE);
        assert_eq!(sim.stats.rejected_requests, 10);
    }
    #[test]
    fn a_close_call_is_counted_per_episode_not_per_frame() {
        let mut sim = Simulation::new(1);
        sim.request(Approach::North, Turn::Right);
        sim.step();
        let mut other = sim.vehicles[0].clone();
        other.id = 100;
        other.distance += 5.0; // envelope gap is about 0.528 m
        sim.vehicles.push(other);
        let positions: Vec<_> = sim.vehicles.iter().map(|v| v.position(&sim.paths)).collect();
        sim.monitor_safety(&positions, &[0.0, 0.0]);
        sim.monitor_safety(&positions, &[0.0, 0.0]);
        assert_eq!(sim.stats.close_calls, 1);
        assert_eq!(sim.stats.collisions, 0);
    }
}
