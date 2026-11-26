use std::time::{Duration, Instant};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::rect::{FRect, Point, Rect};
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::rwops::RWops;
use sdl2::surface::Surface;
use sdl2::video::{Window, WindowContext};
use smart_road::config::*;
use smart_road::geometry::{Approach, Vec2};
use smart_road::simulation::Simulation;

const WIDTH: u32 = 1200;
const HEIGHT: u32 = 928;
const WORLD_PIXELS: u32 = 840;
const LEFT: f64 = 24.0;
const TOP: f64 = 64.0;
const SCALE: f64 = WORLD_PIXELS as f64 / (2.0 * WORLD_HALF);
const BG: Color = Color::RGB(11, 19, 29);
const PANEL: Color = Color::RGB(19, 31, 44);
const WHITE: Color = Color::RGB(227, 237, 242);
const MUTED: Color = Color::RGB(137, 159, 174);
const CYAN: Color = Color::RGB(83, 214, 219);
const GOLD: Color = Color::RGB(240, 190, 95);
const RED: Color = Color::RGB(240, 119, 117);

struct Assets<'a> { road: Texture<'a>, cars: Vec<Texture<'a>> }
fn bmp_texture<'a>(creator: &'a TextureCreator<WindowContext>, bytes: &[u8], transparent: bool) -> Result<Texture<'a>, String> {
    let mut rw = RWops::from_bytes(bytes)?;
    let mut surface = Surface::load_bmp_rw(&mut rw)?;
    if transparent { surface.set_color_key(true, Color::RGB(255, 0, 255))?; }
    let mut texture = creator.create_texture_from_surface(&surface).map_err(|e| e.to_string())?;
    texture.set_blend_mode(BlendMode::Blend);
    Ok(texture)
}
impl<'a> Assets<'a> {
    fn load(creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        let road = bmp_texture(creator, include_bytes!("../assets/road.bmp"), false)?;
        let mut cars = Vec::new();
        for bytes in [
            &include_bytes!("../assets/car_cyan.bmp")[..],
            &include_bytes!("../assets/car_amber.bmp")[..],
            &include_bytes!("../assets/car_coral.bmp")[..],
            &include_bytes!("../assets/car_lime.bmp")[..],
        ] { cars.push(bmp_texture(creator, bytes, true)?); }
        Ok(Self { road, cars })
    }
}
fn screen(p: Vec2) -> Point {
    Point::new((LEFT + (p.x + WORLD_HALF) * SCALE).round() as i32, (TOP + (p.y + WORLD_HALF) * SCALE).round() as i32)
}
fn rect(canvas: &mut Canvas<Window>, x: i32, y: i32, w: u32, h: u32, color: Color) -> Result<(), String> {
    canvas.set_draw_color(color); canvas.fill_rect(Rect::new(x, y, w, h))
}
fn outline_circle(canvas: &mut Canvas<Window>, centre: Vec2, radius: f64, color: Color) -> Result<(), String> {
    canvas.set_draw_color(color);
    let pts: Vec<Point> = (0..=36).map(|k| {
        let a = std::f64::consts::TAU * k as f64 / 36.0;
        screen(centre + Vec2::new(a.cos(), a.sin()) * radius)
    }).collect();
    canvas.draw_lines(pts.as_slice())
}
fn metric(canvas: &mut Canvas<Window>, y: i32, label: &str, value: &str, color: Color) -> Result<(), String> {
    rect(canvas, 888, y, 288, 67, PANEL)?;
    text(canvas, 902, y + 10, label, 1, MUTED)?;
    text(canvas, 902, y + 29, value, 3, color)
}
fn value(n: Option<f64>) -> String { n.map_or_else(|| "N/A".into(), |n| format!("{n:.2}")) }
fn draw_world(canvas: &mut Canvas<Window>, assets: &Assets<'_>, sim: &Simulation, debug: bool, paused: bool) -> Result<(), String> {
    canvas.set_draw_color(BG); canvas.clear();
    text(canvas, 24, 17, "SMART ROAD", 4, WHITE)?;
    text(canvas, 322, 20, "AUTONOMOUS INTERSECTION", 2, CYAN)?;
    text(canvas, 322, 40, "NO LIGHTS / GEOMETRIC RESERVATIONS", 1, MUTED)?;
    canvas.copy(&assets.road, None, Some(Rect::new(LEFT as i32, TOP as i32, WORLD_PIXELS, WORLD_PIXELS)))?;
    canvas.set_clip_rect(Some(Rect::new(LEFT as i32, TOP as i32, WORLD_PIXELS, WORLD_PIXELS)));
    if debug {
        for p in &sim.paths {
            let color = match p.turn as usize { 0 => Color::RGBA(83, 214, 219, 70), 1 => Color::RGBA(240, 190, 95, 70), _ => Color::RGBA(194, 157, 247, 70) };
            canvas.set_draw_color(color);
            let points: Vec<_> = p.control_polyline().into_iter().map(screen).collect();
            canvas.draw_lines(points.as_slice())?;
        }
    }
    for v in &sim.vehicles {
        let (p, tangent) = sim.paths[v.route].pose(v.distance);
        if debug {
            outline_circle(canvas, p, BODY_RADIUS + SAFETY_DISTANCE / 2.0,
                if v.has_reservation(&sim.paths) { Color::RGBA(83, 214, 219, 130) } else { Color::RGBA(240, 190, 95, 100) })?;
        }
        let w = (CAR_LENGTH * SCALE) as f32;
        let h = (CAR_WIDTH * SCALE) as f32;
        let x = (LEFT + (p.x + WORLD_HALF) * SCALE) as f32;
        let y = (TOP + (p.y + WORLD_HALF) * SCALE) as f32;
        // Source sprite points east; positive SDL rotation is clockwise.
        let angle = tangent.y.atan2(tangent.x).to_degrees();
        canvas.copy_ex_f(&assets.cars[v.sprite], None, Some(FRect::new(x - w / 2.0, y - h / 2.0, w, h)), angle, None, false, false)?;
        if debug { text(canvas, x as i32 + 13, y as i32 - 13, &v.id.to_string(), 1, WHITE)?; }
    }
    canvas.set_clip_rect(None);
    text(canvas, 890, 72, "LIVE METRICS", 2, WHITE)?;
    metric(canvas, 101, "VEHICLES PASSED", &sim.stats.completed.to_string(), CYAN)?;
    metric(canvas, 178, "ACTIVE / PENDING REQUESTS", &format!("{} / {}", sim.vehicles.len(), sim.pending_count()), WHITE)?;
    metric(canvas, 255, "ACTIVE RESERVATIONS", &sim.active_reservations().to_string(), GOLD)?;
    metric(canvas, 332, "SIMULATION TIME - SECONDS", &format!("{:.1}", sim.time()), WHITE)?;
    text(canvas, 902, 420, &format!("MAX SPEED  {} M/S", value(sim.stats.max_velocity)), 2, MUTED)?;
    text(canvas, 902, 446, &format!("CLOSE CALLS  {}", sim.stats.close_calls), 2, if sim.stats.close_calls == 0 { CYAN } else { RED })?;
    text(canvas, 902, 472, &format!("COLLISIONS   {}", sim.stats.collisions), 2, if sim.stats.collisions == 0 { MUTED } else { RED })?;
    text(canvas, 902, 514, "APPROACH      ACTIVE", 1, MUTED)?;
    for (i, a) in Approach::ALL.iter().enumerate() {
        let count = sim.vehicles.iter().filter(|v| v.route / 3 == *a as usize).count();
        text(canvas, 902, 537 + i as i32 * 22, a.name(), 2, WHITE)?;
        text(canvas, 1118, 537 + i as i32 * 22, &count.to_string(), 2, CYAN)?;
    }
    rect(canvas, 888, 643, 288, 258, PANEL)?;
    text(canvas, 902, 659, "CONTROLS", 2, WHITE)?;
    for (i, line) in ["ARROWS   REQUEST VEHICLE", "R        RANDOM TRAFFIC", "D        DEBUG OVERLAY", "SPACE    PAUSE / RESUME", "ESC      END + STATISTICS"].iter().enumerate() {
        text(canvas, 902, 696 + i as i32 * 23, line, 1, MUTED)?;
    }
    text(canvas, 902, 831, if sim.auto { "RANDOM: ON" } else { "RANDOM: OFF" }, 2, if sim.auto { CYAN } else { MUTED })?;
    text(canvas, 902, 865, if paused { "PAUSED" } else { "RUNNING / 120 HZ" }, 2, if paused { GOLD } else { WHITE })?;
    if paused {
        rect(canvas, 284, 449, 324, 70, Color::RGBA(11, 19, 29, 230))?;
        text(canvas, 372, 471, "PAUSED", 3, GOLD)?;
    }
    canvas.present();
    Ok(())
}

pub fn run(seed: u64, demo: bool) -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let mut events = sdl.event_pump()?;
    let sim = simulate(&video, &mut events, seed, demo)?;
    print!("{}", sim.stats.report(sim.time(), sim.vehicles.len(), sim.pending_count()));
    statistics_window(&video, &mut events, &sim)
}
fn simulate(video: &sdl2::VideoSubsystem, events: &mut sdl2::EventPump, seed: u64, demo: bool) -> Result<Simulation, String> {
    let window = video.window("Smart Road - Autonomous Intersection", WIDTH, HEIGHT)
        .position_centered().resizable().allow_highdpi().build().map_err(|e| e.to_string())?;
    let mut canvas = window.into_canvas().present_vsync().build().map_err(|e| e.to_string())?;
    canvas.set_logical_size(WIDTH, HEIGHT).map_err(|e| e.to_string())?;
    canvas.set_blend_mode(BlendMode::Blend);
    let creator = canvas.texture_creator();
    let assets = Assets::load(&creator)?;
    let mut sim = Simulation::new(seed);
    sim.set_auto(demo);
    let mut previous = Instant::now();
    let mut accumulator = 0.0;
    let mut debug = false;
    let mut paused = false;
    'running: loop {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } | Event::KeyDown { keycode: Some(Keycode::Escape), repeat: false, .. } => break 'running,
                Event::KeyDown { keycode: Some(Keycode::Up), .. } => sim.request_random_from(Approach::South),
                Event::KeyDown { keycode: Some(Keycode::Down), .. } => sim.request_random_from(Approach::North),
                Event::KeyDown { keycode: Some(Keycode::Right), .. } => sim.request_random_from(Approach::West),
                Event::KeyDown { keycode: Some(Keycode::Left), .. } => sim.request_random_from(Approach::East),
                Event::KeyDown { keycode: Some(Keycode::R), repeat: false, .. } => sim.set_auto(!sim.auto),
                Event::KeyDown { keycode: Some(Keycode::D), repeat: false, .. } => debug = !debug,
                Event::KeyDown { keycode: Some(Keycode::Space), repeat: false, .. } => paused = !paused,
                _ => {}
            }
        }
        let now = Instant::now();
        // Keep UI responsive after a long OS stall; simulated time then runs
        // slower than wall time rather than doing an unbounded catch-up loop.
        accumulator += (now - previous).as_secs_f64().min(0.25);
        previous = now;
        if paused { accumulator = 0.0; }
        while accumulator >= DT {
            sim.step(); accumulator -= DT;
        }
        draw_world(&mut canvas, &assets, &sim, debug, paused)?;
        std::thread::sleep(Duration::from_millis(1));
    }
    // Locals drop here: all textures and the simulation window are destroyed
    // before statistics_window creates a separate, real SDL2 window.
    Ok(sim)
}
fn statistics_window(video: &sdl2::VideoSubsystem, events: &mut sdl2::EventPump, sim: &Simulation) -> Result<(), String> {
    let window = video.window("Smart Road - Final Statistics", 960, 826)
        .position_centered().resizable().allow_highdpi().build().map_err(|e| e.to_string())?;
    let mut canvas = window.into_canvas().present_vsync().build().map_err(|e| e.to_string())?;
    canvas.set_logical_size(960, 826).map_err(|e| e.to_string())?;
    let s = &sim.stats;
    let rows = [
        ("VEHICLES PASSED - TOTAL", s.completed.to_string()),
        ("MAX VELOCITY - M/S", value(s.max_velocity)),
        ("MIN VELOCITY - M/S", value(s.min_velocity)),
        ("MAX PASS TIME - SECONDS", value(s.max_pass_time)),
        ("MIN PASS TIME - SECONDS", value(s.min_pass_time)),
        ("CLOSE-CALL EPISODES", s.close_calls.to_string()),
        ("COLLISION EPISODES", s.collisions.to_string()),
        ("MEAN PASS TIME - SECONDS", value(s.average_pass_time())),
        ("MIN MOVING VELOCITY - M/S", value(s.min_moving_velocity)),
        ("PEAK SIMULTANEOUS RESERVATIONS", s.peak_reservations.to_string()),
        ("HARD-BRAKE OVERRIDES", s.hard_brakes.to_string()),
        ("ACTIVE / PENDING AT EXIT", format!("{} / {}", sim.vehicles.len(), sim.pending_count())),
        ("REJECTED REQUESTS - QUEUE FULL", s.rejected_requests.to_string()),
        ("SIMULATED TIME - SECONDS", format!("{:.2}", sim.time())),
    ];
    loop {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } | Event::KeyDown { keycode: Some(Keycode::Escape | Keycode::Return), repeat: false, .. } => return Ok(()),
                _ => {}
            }
        }
        canvas.set_draw_color(BG); canvas.clear();
        text(&mut canvas, 40, 28, "SIMULATION COMPLETE", 4, WHITE)?;
        text(&mut canvas, 42, 70, "SMART ROAD / DETECTION TO FULL CANVAS EXIT", 2, CYAN)?;
        for (i, (label, val)) in rows.iter().enumerate() {
            let y = 116 + i as i32 * 42;
            rect(&mut canvas, 32, y, 896, 36, if i % 2 == 0 { PANEL } else { BG })?;
            text(&mut canvas, 46, y + 11, label, 2, MUTED)?;
            text(&mut canvas, 750, y + 8, val, 3, if i < 6 { CYAN } else { WHITE })?;
        }
        text(&mut canvas, 42, 734, "MIN VELOCITY INCLUDES STOPS. UNFINISHED CARS ARE NOT PASSES.", 1, MUTED)?;
        text(&mut canvas, 42, 753, "CLOSE CALLS COUNT PAIR EPISODES, NOT EVERY FRAME.", 1, MUTED)?;
        text(&mut canvas, 42, 785, "ESC / ENTER TO CLOSE", 2, GOLD)?;
        canvas.present();
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Tiny in-source 5x7 glyphs: no SDL_ttf extension or external font is needed.
fn text(canvas: &mut Canvas<Window>, x: i32, y: i32, content: &str, scale: u32, color: Color) -> Result<(), String> {
    canvas.set_draw_color(color);
    let mut left = x;
    for ch in content.chars() {
        let rows = glyph(ch.to_ascii_uppercase());
        for (row, bits) in rows.iter().enumerate() {
            for col in 0..5 {
                if bits & (1 << (4 - col)) != 0 {
                    canvas.fill_rect(Rect::new(left + col * scale as i32, y + row as i32 * scale as i32, scale, scale))?;
                }
            }
        }
        left += 6 * scale as i32;
    }
    Ok(())
}
fn glyph(ch: char) -> [u8; 7] {
    match ch {
        'A' => [14,17,17,31,17,17,17], 'B' => [30,17,17,30,17,17,30],
        'C' => [14,17,16,16,16,17,14], 'D' => [30,17,17,17,17,17,30],
        'E' => [31,16,16,30,16,16,31], 'F' => [31,16,16,30,16,16,16],
        'G' => [14,17,16,23,17,17,15], 'H' => [17,17,17,31,17,17,17],
        'I' => [14,4,4,4,4,4,14], 'J' => [7,2,2,2,2,18,12],
        'K' => [17,18,20,24,20,18,17], 'L' => [16,16,16,16,16,16,31],
        'M' => [17,27,21,21,17,17,17], 'N' => [17,25,21,19,17,17,17],
        'O' => [14,17,17,17,17,17,14], 'P' => [30,17,17,30,16,16,16],
        'Q' => [14,17,17,17,21,18,13], 'R' => [30,17,17,30,20,18,17],
        'S' => [15,16,16,14,1,1,30], 'T' => [31,4,4,4,4,4,4],
        'U' => [17,17,17,17,17,17,14], 'V' => [17,17,17,17,17,10,4],
        'W' => [17,17,17,21,21,21,10], 'X' => [17,17,10,4,10,17,17],
        'Y' => [17,17,10,4,4,4,4], 'Z' => [31,1,2,4,8,16,31],
        '0' => [14,17,19,21,25,17,14], '1' => [4,12,4,4,4,4,14],
        '2' => [14,17,1,2,4,8,31], '3' => [30,1,1,14,1,1,30],
        '4' => [2,6,10,18,31,2,2], '5' => [31,16,16,30,1,1,30],
        '6' => [14,16,16,30,17,17,14], '7' => [31,1,2,4,8,8,8],
        '8' => [14,17,17,14,17,17,14], '9' => [14,17,17,15,1,1,14],
        '.' => [0,0,0,0,0,6,6], ',' => [0,0,0,0,6,6,4],
        ':' => [0,6,6,0,6,6,0], '/' => [1,2,2,4,8,8,16],
        '-' => [0,0,0,31,0,0,0], '+' => [0,4,4,31,4,4,0],
        '=' => [0,0,31,0,31,0,0], '_' => [0,0,0,0,0,0,31],
        '(' => [2,4,8,8,8,4,2], ')' => [8,4,2,2,2,4,8],
        ' ' => [0;7], _ => [14,17,1,2,4,0,4],
    }
}
