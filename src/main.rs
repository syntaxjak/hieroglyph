use rand::{rngs::OsRng, Rng, RngCore};
use std::env;
use std::f32::consts::PI;
use std::fs::File;
use std::io::{stdout, BufWriter, Write};
use std::collections::VecDeque;
use std::thread::sleep;
use std::time::Duration;

const WIDTH: usize = 80;
const HEIGHT: usize = 24;
const RESET: &str = "\x1b[0m";
const COLOR_TITLE: &str = "\x1b[38;5;120m";
const COLOR_AURA: &str = "\x1b[38;5;135m";
const COLOR_DIM_STAR: &str = "\x1b[38;5;240m";
const COLOR_BRIGHT_STAR: &str = "\x1b[38;5;45m";
const NUMBER_COLORS: [&str; 6] = [
    "\x1b[38;5;51m",
    "\x1b[38;5;81m",
    "\x1b[38;5;159m",
    "\x1b[38;5;219m",
    "\x1b[38;5;214m",
    "\x1b[38;5;190m",
];

#[derive(Clone, Copy)]
struct Cell {
    ch: char,
    color: Option<&'static str>,
}

#[derive(Clone)]
struct Particle {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,
    life: f32,
    symbol: char,
    color: &'static str,
}

#[derive(Clone)]
struct Star {
    x: usize,
    y: usize,
    speed: f32,
    offset: f32,
}

struct CursorGuard;

impl Drop for CursorGuard {
    fn drop(&mut self) {
        print!("\x1b[?25h");
        let _ = stdout().flush();
    }
}

struct PadWriter {
    hex: BufWriter<File>,
    glyphs: BufWriter<File>,
    count: usize,
}

impl PadWriter {
    fn new(hex_path: &str, glyph_path: &str) -> std::io::Result<Self> {
        let hex_file = File::create(hex_path)?;
        let glyph_file = File::create(glyph_path)?;
        Ok(Self {
            hex: BufWriter::new(hex_file),
            glyphs: BufWriter::new(glyph_file),
            count: 0,
        })
    }

    fn push_byte(&mut self, byte: u8) -> std::io::Result<()> {
        let idx = self.count;
        write!(self.hex, "{:02x}", byte)?;
        write!(self.glyphs, "{}", glyph_from_byte(byte))?;
        self.count += 1;
        if (idx + 1) % 32 == 0 {
            self.hex.write_all(b"\n")?;
            self.glyphs.write_all(b"\n")?;
        }
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<usize> {
        if self.count % 32 != 0 {
            self.hex.write_all(b"\n")?;
            self.glyphs.write_all(b"\n")?;
        }
        self.hex.flush()?;
        self.glyphs.flush()?;
        Ok(self.count)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.hex.flush()?;
        self.glyphs.flush()
    }
}

struct PadWall {
    rows: VecDeque<Vec<u8>>,
    max_rows: usize,
}

impl PadWall {
    fn new(max_rows: usize) -> Self {
        Self {
            rows: VecDeque::new(),
            max_rows,
        }
    }

    fn push(&mut self, byte: u8) {
        if self.rows.back().map(|r| r.len() >= WIDTH).unwrap_or(true) {
            self.rows.push_back(Vec::new());
        }
        if let Some(last) = self.rows.back_mut() {
            last.push(byte);
        }
        while self.rows.len() > self.max_rows {
            self.rows.pop_front();
        }
    }

    fn render(&self, buffer: &mut [Cell]) {
        let rows_available = self.rows.len();
        let pad_rows = self.max_rows.min(HEIGHT.saturating_sub(2));
        let offset = pad_rows.saturating_sub(rows_available);

        for row in 0..pad_rows {
            let y = (HEIGHT - pad_rows) as isize + row as isize;
            if row < offset {
                continue;
            }
            let src_row = row - offset;
            if let Some(data) = self.rows.get(src_row) {
                for (col, byte) in data.iter().enumerate() {
                    let glyph = glyph_from_byte(*byte);
                    let color = NUMBER_COLORS[(*byte as usize) % NUMBER_COLORS.len()];
                    set_cell(buffer, col as isize, y, glyph, Some(color));
                }
            }
        }
    }
}

fn set_cell(buffer: &mut [Cell], x: isize, y: isize, ch: char, color: Option<&'static str>) {
    if x < 0 || y < 0 {
        return;
    }

    let tx = x as usize;
    let ty = y as usize;
    if tx < WIDTH && ty < HEIGHT {
        buffer[ty * WIDTH + tx] = Cell { ch, color };
    }
}

fn render_frame(buffer: &[Cell]) {
    let mut out = String::with_capacity(WIDTH * HEIGHT * 4 + HEIGHT);
    let mut current_color: Option<&'static str> = None;

    for row in 0..HEIGHT {
        let start = row * WIDTH;
        let end = start + WIDTH;

        for cell in &buffer[start..end] {
            if cell.color != current_color {
                match cell.color {
                    Some(color) => out.push_str(color),
                    None => out.push_str(RESET),
                }

                current_color = cell.color;
            }

            out.push(cell.ch);
        }

        if current_color.is_some() {
            out.push_str(RESET);
            current_color = None;
        }

        out.push('\n');
    }

    print!("\x1b[2J\x1b[H{}", out);
    let _ = stdout().flush();
}

fn draw_starfield(buffer: &mut [Cell], stars: &[Star], frame: usize) {
    for star in stars {
        let phase = ((frame as f32) * star.speed + star.offset).sin() * 0.5 + 0.5;
        let (ch, color) = if phase > 0.8 {
            ('✦', Some(COLOR_BRIGHT_STAR))
        } else if phase > 0.45 {
            ('*', Some(COLOR_DIM_STAR))
        } else {
            ('·', Some(COLOR_DIM_STAR))
        };

        set_cell(buffer, star.x as isize, star.y as isize, ch, color);
    }
}

fn draw_swirl(buffer: &mut [Cell], center_x: f32, center_y: f32, frame: usize) {
    let ring_points = 14;
    let base_angle = frame as f32 * 0.18;

    for i in 0..ring_points {
        let angle = base_angle + i as f32 * (2.0 * PI / ring_points as f32);
        let radius = 2.6 + ((frame as f32) * 0.08 + i as f32 * 0.35).sin() * 0.5;
        let px = center_x + angle.cos() * radius;
        let py = center_y + angle.sin() * (radius * 0.6) - 0.4;
        let ch = if (frame + i) % 3 == 0 { '°' } else { '·' };

        set_cell(
            buffer,
            px.round() as isize,
            py.round() as isize,
            ch,
            Some(COLOR_AURA),
        );
    }
}

fn glyph_from_byte(byte: u8) -> char {
    let codepoint = 0x0100 + byte as u32;
    char::from_u32(codepoint).unwrap_or('·')
}

fn print_usage() {
    println!("Usage:");
    println!("  bytefall                 # Run Bytefall, generating an endless pad to pad.hex");
    println!("  bytefall -length <n>     # Run Bytefall until it writes n pad bytes to pad.hex/pad.txt");
}

fn run_animation(mut rng: OsRng, target_len: Option<usize>) -> std::io::Result<()> {
    let mut particles: Vec<Particle> = Vec::new();
    let mut lane_index: usize = 0;
    let lane_count: usize = 24;
    let stream_center_x = WIDTH as f32 / 2.0;
    let swirl_center_y = 6.0;
    let pad_rows: usize = HEIGHT.saturating_sub(2);
    let mut pad_wall = PadWall::new(pad_rows);
    let mut pad_writer = PadWriter::new("pad.hex", "pad.txt")?;
    let mut completion_frames: usize = 0;

    let mut stars: Vec<Star> = Vec::new();
    for _ in 0..55 {
        let x = (rng.next_u32() as usize) % WIDTH;
        let y = (rng.next_u32() as usize) % (HEIGHT.saturating_sub(6).max(1));
        stars.push(Star {
            x,
            y,
            speed: rng.gen_range(0.02..0.06),
            offset: rng.gen_range(0.0..(2.0 * PI)),
        });
    }

    let _cursor = CursorGuard;
    println!("\x1b[?25l");

    let mut frame: usize = 0;
    loop {
        let pad_full = target_len.map(|t| pad_writer.count >= t).unwrap_or(false);

        if particles.len() < 48 && !pad_full {
            let lane_offset = (lane_index as f32) - (lane_count as f32 / 2.0) + 0.5;
            lane_index = (lane_index + 1) % lane_count;

            let byte = rng.next_u32() as u8;
            let symbol = glyph_from_byte(byte);
            let color = NUMBER_COLORS[(byte as usize) % NUMBER_COLORS.len()];
            let spawn_x = stream_center_x + lane_offset * 1.4 + rng.gen_range(-0.3..0.3);
            let spawn_y = 1.0;
            let vx = rng.gen_range(-0.08..0.08);
            let vy = rng.gen_range(0.35..0.95);

            pad_writer.push_byte(byte)?;
            pad_wall.push(byte);

            particles.push(Particle {
                x: spawn_x,
                y: spawn_y,
                vx,
                vy,
                life: rng.gen_range(2.4..3.4),
                symbol,
                color,
            });
        }

        if pad_full {
            completion_frames += 1;
            if completion_frames > 90 {
                pad_writer.finish()?;
                break;
            }
        }

        for p in &mut particles {
            p.x += p.vx;
            p.y += p.vy;
            p.vx *= 0.995;
            p.vy += 0.018;
            p.life -= 0.02;
        }

        particles.retain(|p| p.life > 0.0 && p.y > 0.0 && p.y < HEIGHT as f32 + 2.0);

        let mut buffer = vec![
            Cell {
                ch: ' ',
                color: None,
            };
            WIDTH * HEIGHT
        ];

        draw_starfield(&mut buffer, &stars, frame);
        draw_swirl(&mut buffer, stream_center_x, swirl_center_y, frame);

        for p in &particles {
            let px = p.x.round() as isize;
            let py = p.y.round() as isize;
            let glyph = if p.life < 0.25 { '·' } else { p.symbol };
            set_cell(&mut buffer, px, py, glyph, Some(p.color));
        }

        pad_wall.render(&mut buffer);

        let title = if let Some(t) = target_len {
            format!("  NINE - Bytefall ({}/{})  ", pad_writer.count.min(t), t)
        } else {
            format!("  NINE - Bytefall ({})  ", pad_writer.count)
        };
        let start = (WIDTH.saturating_sub(title.len())) / 2;
        for (idx, ch) in title.chars().enumerate() {
            set_cell(
                &mut buffer,
                (idx + start) as isize,
                0,
                ch,
                Some(COLOR_TITLE),
            );
        }

        render_frame(&buffer);
        pad_writer.flush()?;
        sleep(Duration::from_millis(55));
        frame = frame.wrapping_add(1);
    }

    Ok(())
}

fn main() {
    let rng = OsRng;

    let mut args = env::args().skip(1);
    let mut pad_length: Option<usize> = None;

    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-length" | "--length" => {
                if let Some(val) = args.next() {
                    match val.parse::<usize>() {
                        Ok(v) => pad_length = Some(v),
                        Err(_) => {
                            eprintln!("Invalid length: {val}");
                            std::process::exit(1);
                        }
                    }
                } else {
                    eprintln!("Missing value for -length");
                    std::process::exit(1);
                }
            }
            "-h" | "--help" => {
                print_usage();
                return;
            }
            other => {
                eprintln!("Unknown argument: {other}");
                print_usage();
                std::process::exit(1);
            }
        }
    }

    if let Err(err) = run_animation(rng, pad_length) {
        eprintln!("Bytefall error: {err}");
    }
}
