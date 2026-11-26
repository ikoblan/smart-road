# Smart Road

A Rust + SDL2 cross-intersection simulation without traffic lights. Twelve
immutable lane routes, geometric conflict reservations, same-route following,
rotating car sprites, keyboard-generated traffic, and a separate statistics
window.

**Verification status:** The source, assets, and 30 Rust test functions are
included. This delivery environment did not have `rustc` or Cargo, so the Rust
project has **not been compiled or executed here**, and the SDL2 interface has
not been interactively verified. The independent Python mathematical-model
checks in `validation/` were executed; they are not substitutes for the Cargo
tests. See `docs/VALIDATION.md` before treating the project as submission-ready.

## Start on Ubuntu

Use a current stable Rust toolchain with Cargo. Then:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libsdl2-dev unzip
unzip smart_road.zip
cd smart_road

# Test the actual Rust core without a window or SDL2 build:
cargo test --no-default-features

# Compile and launch the GUI:
cargo run --release
```

The SDL2 Rust binding is pinned to `0.38.0`. The default features build and
statically link bundled SDL2. This avoids relying on an older system SDL2 on
LTS distributions. A C compiler and CMake are needed for that build. The first
Cargo invocation also needs Internet access to resolve/download Rust packages.

`libsdl2-dev` is useful for the native Linux development dependencies and for
the optional system-library build below. The default executable does not link
against the system SDL2 shared library. No SDL2_image, SDL2_ttf, external font,
or runtime asset folder is needed: BMP textures are embedded with
`include_bytes!`, and the UI's small glyphs are drawn directly in Rust.

```bash
# Auto-generation enabled at startup:
cargo run --release -- --demo

# Headless simulation; 120 simulated seconds and deterministic seed 42:
cargo run --release --no-default-features -- --headless 120 42

# Use installed SDL2 instead of building the bundled library:
# The binding's maintainers recommend system SDL2 >= 2.0.26.
cargo run --release --no-default-features --features gui

# Run the complete test suite with default build features:
cargo test

# Format the generated source with the Rust formatter:
cargo fmt --all
```

Cargo will create `Cargo.lock` on its first successful dependency resolution.
Commit that generated file for the application. No lockfile was fabricated in
this delivery, and transitive dependency versions are not pinned without it.

## Controls

| Key | Action |
| --- | --- |
| Up | Request a vehicle entering from the **south**, initially heading north |
| Down | Request a vehicle entering from the **north**, initially heading south |
| Right | Request a vehicle entering from the **west**, initially heading east |
| Left | Request a vehicle entering from the **east**, initially heading west |
| R | Toggle continuous random arrivals in the simulation loop |
| D | Toggle route traces, vehicle IDs, and conservative safety envelopes |
| Space | Pause/resume simulated time |
| Esc | End the simulation and open a **new SDL2 statistics window** |
| Esc / Enter in statistics | Close the program |

Each arrow press selects a random right/straight/left route **on the selected
approach**. It does not force every car to go straight. The route never changes
after selection. Arrow-key repeat is allowed, but requests are queued and cars
are spawned only when the spawn interval and spatial clearance both allow it.
There is a bounded queue of 64 pending requests per route; excess requests are
rejected and counted explicitly rather than silently consuming unbounded RAM.

Random mode requests one vehicle every **2.5 simulated seconds** by default
(0.4 requests/second). This is a demonstration load, not a claimed capacity.
The heavy-traffic tests request vehicles twice as fast. Change `AUTO_INTERVAL`
in `src/config.rs` to experiment with demand and congestion.

## How the intersection works

There are four approaches with three inbound lanes each. North is the template
route geometry; quarter rotations produce east, south, and west. Coordinates
are metres, with positive x to the right and positive y down, matching SDL2.
The canvas scale is 7 pixels/metre.

Every trajectory is an incoming straight segment, an exact circular quarter
arc (or a straight crossing), and an outgoing straight segment. Position is
parameterized by **distance along the route**, not a raw Bézier parameter.
The car's rotation comes from the path tangent. The dedicated outgoing lane
for each maneuver avoids uncontrolled merges outside the intersection.

The controller builds a 12 x 12 conflict matrix from the geometric corridors.
It considers the car's enclosing disc, the positive safety distance, and a
margin greater than the curve-approximation error. Crossing corridors cannot
hold reservations together. Geometrically compatible corridors can: four
right turns, for example, can be admitted concurrently.

Cars on the **same route** may share its corridor, with an independent following
controller enforcing an 8 m arc-length headway. This avoids serializing an
entire lane until its previous car has disappeared. Different conflicting
routes wait until all conflicting reservation holders have crossed the release
boundary. A car never relinquishes its reservation while it is still in the
protected crossing region.

The oldest blocked request prevents younger conflicting requests from jumping
ahead. Only that oldest blocked request forms the fairness barrier; chaining
every blocked route would unnecessarily suppress compatible traffic. This is
an event-driven reservation policy, not periodic red/green phases.

This controller is intentionally conservative. It reserves whole corridors,
not optimally packed space-time tiles. Safety and eventual service are the
design priorities; **global minimum delay or maximum throughput is not proven**.
Sustained arrivals above service capacity will create a visible backlog.

See `docs/ALGORITHM.md` for equations, geometric assumptions, and invariants.

## Vehicle physics

Default setpoints are 12 m/s in free flow, 10 m/s through a straight crossing,
6 m/s on a left turn, and 4 m/s on a right turn. Waiting may produce 0 m/s.
Acceleration/deceleration make actual speeds continuous rather than limited
to these four setpoints.

All decisions use the same pre-step snapshot. At a fixed 120 Hz:

```text
v_next = bounded change toward the target speed
s_next = s + v_next * dt
velocity = distance / time = (s_next - s) / dt
```

A stationary-obstacle stopping bound anticipates both the next update and
braking distance. There is also a last-resort hard position limit; an override
is counted in statistics instead of being misrepresented as smooth braking.
The shipped scenarios assert zero hard-brake overrides.

The safety monitor is independent of the scheduler. It checks relative swept
motion across the entire step, with an arc-deviation margin, instead of only
checking the two end-of-frame positions. It records conservative disc-envelope
violations; the enclosing discs are deliberately larger than the car sprites.

## Statistics and their definitions

The final SDL2 window and terminal report include the requested cumulative
number of completed vehicles, highest/lowest observed speed, longest/shortest
completed passage, and close calls. The audit's "max number of vehicles that
passed" is interpreted as the cumulative completed count: four finished cars
produce 4, not a frame maximum or the number still visible.

A vehicle is first detected when it is **actually spawned** into the simulation.
Its timer starts then, includes approach and queue waiting, and ends when its
whole conservative body envelope leaves the road canvas. The last tick's exit
instant is interpolated. Unspawned keyboard requests are not yet AVs and are
reported separately as pending requests. Stopping with Esc does not count
unfinished cars as completed passages.

Speed extrema include all spawned vehicles and their waiting speeds. The
required minimum therefore includes zero when a stop occurs. Minimum moving
speed is also shown separately. Passage-time extrema include completed cars
only. Unobserved extrema are `N/A`, not invented zeros or infinity.

Close calls count **pair episodes**: a pair continuously violating the safety
threshold for 100 frames adds one episode, not 100. A later separate violation
by the same pair counts again. Equality at the safety threshold is unsafe, as
required by the unit-test instructions. Collision episodes, mean travel time,
peak active cars, peak reservations, queue rejections, and hard-brake overrides
are additional statistics.

Time is simulated time, not an unconditional wall-clock stopwatch. Pauses do
not count. Under long operating-system stalls the GUI limits catch-up work and
the simulation can run slower than wall time.

## Project layout

```text
src/
  main.rs          CLI and optional GUI entry point
  lib.rs           Display-independent Rust core
  config.rs        SI-unit constants and generation rate
  geometry.rs      Twelve analytic routes and distance geometry
  physics.rs       Velocity, proximity, stopping bound, integration
  controller.rs    Corridor conflicts, admission, fairness
  vehicle.rs       Per-vehicle state and reservation lifetime
  simulation.rs    Queues, spawning, snapshots, motion, safety monitor
  stats.rs         Accumulation and textual report
  ui.rs            SDL2 events, texture rotation, HUD, statistics window
assets/            Five original BMP textures, embedded at compile time
tests/scenarios.rs Rust scenario/integration tests
tools/             Optional asset regeneration and local verification scripts
validation/        Independent Python reference check and its actual output
docs/              Algorithm explanation and verification status
```

## Tests and submission review

There are **30 Rust test functions**, including one that iterates over **all
144 ordered pairs** of the twelve routes. Tests cover velocity calculations,
invalid inputs, the inclusive safety boundary, swept detection, route geometry,
compatible parallel admission, yielding on a left/straight conflict, fairness,
spam queues, statistics, a 36-car burst, five seeded heavy-traffic scenarios,
timer semantics, and random generation cadence.

Before submitting, run the Cargo tests and build, then manually exercise every
keyboard mapping, car orientation through both turns, spam prevention, and the
separate statistics window. Run R for at least a minute. Generate exactly four
cars, wait for all to leave, and verify the final count is 4. Repeat with a
single car and check that minimum and maximum passage time agree.

The Rust core tests do not verify SDL2 event delivery, graphics-driver support,
window resizing, or keyboard-repeat behavior on your operating system. Those
need the manual GUI checks. `tools/verify.sh` performs the command-line checks
but does not replace that interactive review.

## Assets and license

The road and four car textures were created for this project. The source
asset-generation script is `tools/make_assets.py`; regenerating the files is
optional and requires Pillow. The application itself needs neither Python nor
Pillow. These assets and the project code are provided under the included MIT
license. Dependency licenses remain their own.

## Reference documentation

The supplied subject is the requirements source. API/build details were checked
against the official Rust-SDL2 documentation and its repository. The public
01-edu audit was also consulted. Reference locations:

```text
https://docs.rs/sdl2/0.38.0/sdl2/
https://github.com/Rust-SDL2/rust-sdl2
https://raw.githubusercontent.com/01-edu/public/master/subjects/smart-road/audit/README.md
```

This is an educational simulator, not a controller validated for real roads.
It assumes perfect state knowledge, instantaneous communication, cooperative
vehicles, no pedestrians, and no mechanical or sensing failures.
