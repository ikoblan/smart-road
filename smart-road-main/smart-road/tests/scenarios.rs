use smart_road::config::*;
use smart_road::controller::ConflictMatrix;
use smart_road::geometry::{all_paths, Approach, Turn};
use smart_road::simulation::Simulation;

fn drain(sim: &mut Simulation, maximum_seconds: f64) {
    sim.set_auto(false);
    for _ in 0..(maximum_seconds / DT) as usize {
        if sim.vehicles.is_empty() && sim.pending_count() == 0 { return; }
        sim.step();
        assert_eq!(sim.stats.collisions, 0, "collision at {:.3} s", sim.time());
        assert_eq!(sim.stats.close_calls, 0, "close call at {:.3} s", sim.time());
    }
    panic!("traffic did not drain: {} active, {} pending", sim.vehicles.len(), sim.pending_count());
}

#[test]
fn all_144_ordered_route_pairs_drain_safely() {
    let paths = all_paths();
    let matrix = ConflictMatrix::build(&paths);
    for a in 0..12 {
        for b in 0..12 {
            let mut sim = Simulation::with_layout(0, paths.clone(), matrix.clone());
            sim.request(paths[a].approach, paths[a].turn);
            sim.request(paths[b].approach, paths[b].turn);
            drain(&mut sim, 90.0);
            assert_eq!(sim.stats.completed, 2, "pair {a}, {b}");
        }
    }
}
#[test]
fn left_turn_versus_straight_causes_a_velocity_adjustment() {
    let mut sim = Simulation::new(9);
    sim.request(Approach::North, Turn::Left);
    sim.request(Approach::East, Turn::Straight);
    let mut observed_yield = false;
    for _ in 0..(45.0 / DT) as usize {
        sim.step();
        observed_yield |= sim.vehicles.iter().any(|v| v.requested_at.is_some() && !v.admitted && v.velocity < FAST_SPEED - 0.1);
    }
    assert!(observed_yield, "at least one conflicting car must slow down");
    assert_eq!(sim.stats.completed, 2);
    assert_eq!(sim.stats.close_calls, 0);
    assert_eq!(sim.stats.collisions, 0);
}
#[test]
fn right_turns_are_not_serialized_as_a_single_global_lock() {
    let mut sim = Simulation::new(2);
    for a in Approach::ALL { sim.request(a, Turn::Right); }
    drain(&mut sim, 60.0);
    assert_eq!(sim.stats.completed, 4);
    assert_eq!(sim.stats.peak_reservations, 4);
}
#[test]
fn three_vehicles_in_each_of_twelve_lanes() {
    let mut sim = Simulation::new(3);
    for a in Approach::ALL { for t in Turn::ALL { for _ in 0..3 { sim.request(a, t); } } }
    drain(&mut sim, 360.0);
    assert_eq!(sim.stats.completed, 36);
    assert_eq!(sim.stats.hard_brakes, 0);
}
#[test]
fn fixed_seed_heavy_traffic_then_complete_drain() {
    let paths = all_paths();
    let matrix = ConflictMatrix::build(&paths);
    for seed in [0, 1, 7, 42, 2026] {
        let mut sim = Simulation::with_layout(seed, paths.clone(), matrix.clone());
        // Deliberately stress at 0.8 requests/s, twice the GUI's default rate.
        for tick in 0..(120.0 / DT) as usize {
            if tick % 150 == 0 { sim.request_random(); }
            sim.step();
        }
        let requested = sim.stats.spawned + sim.pending_count() as u64;
        drain(&mut sim, 600.0);
        assert_eq!(sim.stats.completed, requested, "seed {seed}");
        assert_eq!(sim.stats.close_calls, 0, "seed {seed}");
        assert_eq!(sim.stats.hard_brakes, 0, "seed {seed}");
    }
}
#[test]
fn finite_old_requests_eventually_finish_under_continued_arrivals() {
    let mut sim = Simulation::new(11);
    sim.request(Approach::North, Turn::Left);
    sim.set_auto(true);
    let mut oldest_spawned_id = None;
    let mut oldest_finished = false;
    for _ in 0..(180.0 / DT) as usize {
        sim.step();
        if oldest_spawned_id.is_none() {
            oldest_spawned_id = sim.vehicles.iter().find(|v| v.route == 2).map(|v| v.id);
        }
        if let Some(id) = oldest_spawned_id {
            if !sim.vehicles.iter().any(|v| v.id == id) { oldest_finished = true; break; }
        }
    }
    assert!(oldest_finished);
    assert_eq!(sim.stats.collisions, 0);
}
#[test]
fn timing_starts_at_detection_and_includes_waiting_until_removal() {
    let mut sim = Simulation::new(5);
    sim.request(Approach::South, Turn::Left);
    drain(&mut sim, 90.0);
    assert_eq!(sim.stats.completed, 1);
    let t = sim.stats.min_pass_time.unwrap();
    assert_eq!(sim.stats.min_pass_time, sim.stats.max_pass_time);
    assert!((sim.time() - t).abs() <= DT + 1e-8);
    assert!(t >= sim.paths[8].length / FAST_SPEED);
}
#[test]
fn no_vehicles_means_no_fake_zero_speed_or_time_statistics() {
    let mut sim = Simulation::new(0);
    for _ in 0..120 { sim.step(); }
    assert_eq!(sim.stats.completed, 0);
    assert_eq!(sim.stats.max_velocity, None);
    assert_eq!(sim.stats.min_pass_time, None);
}

#[test]
fn random_mode_uses_the_game_loop_and_respects_its_interval() {
    let mut sim = Simulation::new(42);
    sim.set_auto(true);
    for _ in 0..(60.0 / DT) as usize { sim.step(); }
    assert_eq!(sim.stats.spawned + sim.pending_count() as u64, 24);
    assert_eq!(sim.stats.close_calls, 0);
    assert_eq!(sim.stats.collisions, 0);
}
