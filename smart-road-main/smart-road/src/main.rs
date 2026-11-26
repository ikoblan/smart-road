use smart_road::config::DT;
use smart_road::simulation::Simulation;
#[cfg(feature = "gui")]
mod ui;

fn main() {
    if let Err(error) = run() { eprintln!("smart_road: {error}"); std::process::exit(1); }
}
fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("--help") | Some("-h") => {
            println!(concat!("smart_road\n",
                "  cargo run --release                      Interactive simulation\n",
                "  cargo run --release -- --demo             Automatic traffic at launch\n",
                "  cargo run --release -- --headless 120 42  120 simulated seconds, seed 42\n",
                "  cargo test --no-default-features          Display-free core tests\n",
                "\nArrows: request a car from the opposite side of the arrow direction\n",
                "R: toggle random traffic   D: safety/path overlay   Space: pause\n",
                "Esc: finish and open statistics in a new window\n"));
            Ok(())
        }
        Some("--headless") => {
            if !(2..=3).contains(&args.len()) { return Err("usage: --headless SECONDS [SEED]".into()); }
            let seconds: f64 = args[1].parse().map_err(|_| "SECONDS must be numeric")?;
            if !seconds.is_finite() || seconds <= 0.0 || seconds > 1_000_000.0 {
                return Err("SECONDS must be finite and in (0, 1000000]".into());
            }
            let seed = args.get(2).map_or(Ok(42_u64), |s| s.parse::<u64>()).map_err(|_| "SEED must be an unsigned integer")?;
            let mut sim = Simulation::new(seed);
            sim.set_auto(true);
            for _ in 0..(seconds / DT).ceil() as u64 { sim.step(); }
            println!("Seed: {seed}");
            print!("{}", sim.stats.report(sim.time(), sim.vehicles.len(), sim.pending_count()));
            Ok(())
        }
        None | Some("--demo") if args.len() <= 1 => {
            #[cfg(feature = "gui")]
            {
                let seed = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
                    .map_err(|e| e.to_string())?.as_nanos() as u64;
                ui::run(seed, !args.is_empty())
            }
            #[cfg(not(feature = "gui"))]
            { Err("GUI is disabled. Build with default features, or use --headless SECONDS [SEED].".into()) }
        }
        _ => Err("unknown arguments; run with --help".into()),
    }
}
