# Verification record

## Executed here

An independent Python/NumPy reference model was executed for the equations,
route geometry, reservation logic, following, and swept-clearance monitoring.
Its source and console output are supplied in `validation/`. It is deliberately
separate from the Rust implementation, so transcription or Rust-specific errors
remain possible.

Recorded mathematical-model checks:

- All twelve route joins were continuous within the tested tolerance.
- There were 50 compatible unordered distinct-route pairs out of 66.
- The smallest compatible-corridor centre clearance was 6.000000 m, versus
  a required 5.472136 m.
- All 144 ordered two-route scenarios drained without a detected safety-gap
  violation; their minimum conservative gap was 1.527419 m.
- Four right-turn requests held four simultaneous reservations.
- A burst of three cars in each of twelve lanes (36 cars) drained in
  71.117 simulated seconds, with minimum conservative gap 1.527352 m.
- Five heavy-traffic runs used seeds 0, 1, 7, 42, and 2026. Each requested
  96 vehicles over 120 seconds (0.8 requests/second), then stopped arrivals
  and drained. Each completed all 96 with no detected close calls, no
  detected collisions, and no hard-brake overrides in the reference model.
- At that heavy rate, total time including drain ranged from 206.008 to
  270.400 seconds. This demonstrates congestion under sustained overload;
  it is not evidence of globally optimized throughput.

The GUI's default auto-arrival rate is lower, 0.4 requests/second. The heavy
scenario intentionally stresses at twice that rate. The independent default
mode check requests 24 cars in 60 simulated seconds; see its separate log.

Assets were regenerated with `tools/make_assets.py`. The road image was visually
inspected, and the BMP dimensions and embedded paths were checked. This is not
an SDL2 GUI smoke test.

To rerun independent checks:

```bash
python3 -m pip install numpy
python3 validation/reference_check.py geometry
python3 validation/reference_check.py pairs
python3 validation/reference_check.py traffic
python3 validation/reference_check.py default
```

## Not executed here

`rustc` and `cargo` were absent in the delivery environment. Therefore:

- Cargo dependency resolution, compilation, linking, and the 30 Rust test
  functions have **not** been executed here.
- The separate SDL2 statistics window, keyboard events, texture loading,
  renderer selection, and interactive animation have **not** been run here.
- No successful build, passing Rust tests, or passed 01-edu audit is claimed.
- No Cargo.lock was invented; generate and commit it after a successful build.

## Required local checks

Run these in the project directory with a stable Rust toolchain installed:

```bash
cargo test --no-default-features
cargo build --release
cargo test
cargo run --release
```

Then exercise all four arrow mappings, repeated key presses, R, D, Space,
both turn animations, Esc, and the final window's close controls. Check four
completed cars give a count of four, and a single car gives equal passage-time
extrema. Verify that no cars are counted as completed merely because Esc was
pressed.

`tools/verify.sh` and the supplied CI workflow run the command-line checks on
a suitable machine. Their presence is not a record that they have passed.
