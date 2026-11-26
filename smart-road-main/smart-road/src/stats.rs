#[derive(Clone, Debug, Default)]
pub struct Statistics {
    pub spawned: u64,
    /// The subject's "max number ... passed" is the cumulative completed count.
    pub completed: u64,
    pub max_velocity: Option<f64>,
    pub min_velocity: Option<f64>,
    pub min_moving_velocity: Option<f64>,
    pub max_pass_time: Option<f64>,
    pub min_pass_time: Option<f64>,
    pub total_pass_time: f64,
    pub total_pass_distance: f64,
    pub total_waiting_time: f64,
    pub close_calls: u64,
    pub collisions: u64,
    pub hard_brakes: u64,
    pub rejected_requests: u64,
    pub peak_active: usize,
    pub peak_pending: usize,
    pub peak_reservations: usize,
    pub min_observed_gap: Option<f64>,
}
impl Statistics {
    pub fn observe_velocity(&mut self, speed: f64) {
        assert!(speed.is_finite() && speed >= 0.0);
        self.max_velocity = Some(self.max_velocity.map_or(speed, |x| x.max(speed)));
        self.min_velocity = Some(self.min_velocity.map_or(speed, |x| x.min(speed)));
        if speed > 1e-3 { self.min_moving_velocity = Some(self.min_moving_velocity.map_or(speed, |x| x.min(speed))); }
    }
    pub fn observe_gap(&mut self, gap: f64) {
        self.min_observed_gap = Some(self.min_observed_gap.map_or(gap, |x| x.min(gap)));
    }
    pub fn record_pass(&mut self, time: f64, distance: f64, waiting: f64) {
        assert!(time.is_finite() && time > 0.0);
        self.completed += 1;
        self.max_pass_time = Some(self.max_pass_time.map_or(time, |x| x.max(time)));
        self.min_pass_time = Some(self.min_pass_time.map_or(time, |x| x.min(time)));
        self.total_pass_time += time;
        self.total_pass_distance += distance;
        self.total_waiting_time += waiting;
    }
    pub fn average_pass_time(&self) -> Option<f64> {
        if self.completed == 0 { None } else { Some(self.total_pass_time / self.completed as f64) }
    }
    pub fn report(&self, sim_time: f64, active: usize, pending: usize) -> String {
        fn value(v: Option<f64>, unit: &str) -> String { v.map_or_else(|| "N/A".into(), |x| format!("{x:.3} {unit}")) }
        format!(concat!(
            "SMART ROAD - FINAL STATISTICS\n",
            "Simulated time: {sim_time:.3} s\n",
            "Vehicles passed (cumulative): {completed}\n",
            "Vehicles spawned: {spawned}\nActive / pending requests: {active} / {pending}\n",
            "Maximum velocity: {max_v}\nMinimum velocity (including stops): {min_v}\n",
            "Minimum moving velocity: {min_moving}\n",
            "Maximum pass time: {max_t}\nMinimum pass time: {min_t}\nMean pass time: {mean_t}\n",
            "Close-call episodes: {close}\nCollision episodes: {collisions}\n",
            "Hard-brake overrides: {hard}\nMinimum conservative gap: {gap}\n",
            "Peak simultaneous reservations: {peak_res}\nPeak active vehicles: {peak_active}\n",
            "Peak pending requests: {peak_pending}\nRejected requests (queue full): {rejected}\n"
        ), sim_time = sim_time, completed = self.completed, spawned = self.spawned,
            active = active, pending = pending, max_v = value(self.max_velocity, "m/s"),
            min_v = value(self.min_velocity, "m/s"), min_moving = value(self.min_moving_velocity, "m/s"),
            max_t = value(self.max_pass_time, "s"), min_t = value(self.min_pass_time, "s"),
            mean_t = value(self.average_pass_time(), "s"), close = self.close_calls,
            collisions = self.collisions, hard = self.hard_brakes,
            gap = value(self.min_observed_gap, "m"), peak_res = self.peak_reservations,
            peak_active = self.peak_active, peak_pending = self.peak_pending,
            rejected = self.rejected_requests)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn extrema_and_completed_count_accumulate() {
        let mut s = Statistics::default();
        assert_eq!(s.min_velocity, None);
        assert_eq!(s.min_pass_time, None);
        for speed in [6.0, 12.0, 0.0, 4.0] { s.observe_velocity(speed); }
        s.record_pass(10.0, 100.0, 1.0);
        s.record_pass(8.0, 96.0, 0.0);
        s.record_pass(20.0, 120.0, 5.0);
        assert_eq!(s.completed, 3);
        assert_eq!(s.max_velocity, Some(12.0));
        assert_eq!(s.min_velocity, Some(0.0));
        assert_eq!(s.min_moving_velocity, Some(4.0));
        assert_eq!(s.max_pass_time, Some(20.0));
        assert_eq!(s.min_pass_time, Some(8.0));
        assert_eq!(s.average_pass_time(), Some(38.0 / 3.0));
    }
    #[test]
    fn a_single_pass_has_identical_time_extrema() {
        let mut s = Statistics::default();
        s.record_pass(7.25, 80.0, 0.0);
        assert_eq!(s.min_pass_time, s.max_pass_time);
    }
}
