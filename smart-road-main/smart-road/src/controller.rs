use crate::config::*;
use crate::geometry::{segment_distance, Path};

#[derive(Clone, Debug)]
pub struct ConflictMatrix { pub conflicts: [[bool; ROUTE_COUNT]; ROUTE_COUNT] }
impl ConflictMatrix {
    pub fn build(paths: &[Path]) -> Self {
        assert_eq!(paths.len(), ROUTE_COUNT);
        let lines: Vec<_> = paths.iter().map(Path::control_polyline).collect();
        let mut conflicts = [[false; ROUTE_COUNT]; ROUTE_COUNT];
        for i in 0..ROUTE_COUNT {
            conflicts[i][i] = true; // identical geometric corridors overlap
            for j in i + 1..ROUTE_COUNT {
                let hit = lines[i].windows(2).any(|a| lines[j].windows(2).any(|b| {
                    segment_distance(a[0], a[1], b[0], b[1]) <= REQUIRED_CENTRE_DISTANCE + CONFLICT_MARGIN
                }));
                conflicts[i][j] = hit;
                conflicts[j][i] = hit;
            }
        }
        Self { conflicts }
    }
    pub fn conflicts(&self, a: usize, b: usize) -> bool { self.conflicts[a][b] }
    /// Candidates arrive in oldest-request-first order. The oldest blocked
    /// request forms a fairness barrier. Younger compatible requests still
    /// pass; barriers are NOT chained, which would suppress useful concurrency.
    /// Same-route cars may share a reservation corridor because the independent
    /// following controller enforces their arc-length headway.
    pub fn admit(&self, candidates: &[(u64, usize)], active_routes: &[usize]) -> Vec<u64> {
        let mut busy = active_routes.to_vec();
        let mut oldest_blocked: Option<usize> = None;
        let mut admitted = Vec::new();
        for &(id, route) in candidates {
            let unavailable = busy.iter().any(|&r| r != route && self.conflicts(route, r))
                || oldest_blocked.map_or(false, |r| self.conflicts(route, r));
            if unavailable {
                if oldest_blocked.is_none() { oldest_blocked = Some(route); }
            } else { busy.push(route); admitted.push(id); }
        }
        admitted
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{all_paths, Approach, Turn};
    #[test]
    fn matrix_is_symmetric_and_routes_conflict_with_themselves() {
        let m = ConflictMatrix::build(&all_paths());
        for i in 0..12 { assert!(m.conflicts(i, i)); for j in 0..12 { assert_eq!(m.conflicts(i, j), m.conflicts(j, i)); } }
    }
    #[test]
    fn crossing_left_turn_and_straight_cannot_both_be_admitted() {
        let paths = all_paths();
        let m = ConflictMatrix::build(&paths);
        let left = Approach::North as usize * 3 + Turn::Left as usize;
        // East-to-west straight crosses the north-to-east left arc.
        let straight = Approach::East as usize * 3 + Turn::Straight as usize;
        assert!(m.conflicts(left, straight));
        assert_eq!(m.admit(&[(1, left), (2, straight)], &[]), vec![1]);
    }
    #[test]
    fn following_cars_can_share_the_same_route_corridor() {
        let m = ConflictMatrix::build(&all_paths());
        assert_eq!(m.admit(&[(2, 0)], &[0]), vec![2]);
    }
    #[test]
    fn four_right_turns_can_be_admitted_together() {
        let m = ConflictMatrix::build(&all_paths());
        assert_eq!(m.admit(&[(1, 0), (2, 3), (3, 6), (4, 9)], &[]), vec![1, 2, 3, 4]);
    }
    #[test]
    fn blocked_older_request_prevents_conflicting_queue_jumping() {
        // Deliberately constructed graph: 0 conflicts with 1; 1 conflicts with
        // 2; 0 and 2 are compatible. Route 2 must not starve waiting route 1.
        let mut m = ConflictMatrix { conflicts: [[false; 12]; 12] };
        for i in 0..12 { m.conflicts[i][i] = true; }
        for (a, b) in [(0, 1), (1, 2)] { m.conflicts[a][b] = true; m.conflicts[b][a] = true; }
        assert!(m.admit(&[(10, 1), (11, 2)], &[0]).is_empty());
        assert_eq!(m.admit(&[(10, 1), (11, 2)], &[]), vec![10]);
    }
}
