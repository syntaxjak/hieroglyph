use fs2::FileExt;
use rand::{rngs::OsRng, Rng, RngCore};
use sha2::{Digest, Sha256};
use std::collections::VecDeque;
use std::env;
use std::f32::consts::PI;
use std::fs::{File, OpenOptions};
use std::io::{self, stdout, BufRead, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
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

const DEFAULT_PAD_PATH: &str = "pad.bin";

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
    writer: BufWriter<File>,
    count: usize,
}

impl PadWriter {
    fn new(glyph_path: &str) -> std::io::Result<Self> {
        let glyph_file = File::create(glyph_path)?;
        Ok(Self {
            writer: BufWriter::new(glyph_file),
            count: 0,
        })
    }

    fn push_byte(&mut self, byte: u8) -> std::io::Result<()> {
        self.writer.write_all(&[byte])?;
        self.count += 1;
        Ok(())
    }

    fn finish(mut self) -> std::io::Result<usize> {
        self.writer.flush()?;
        Ok(self.count)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
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

fn byte_from_glyph(ch: char) -> Option<u8> {
    let code = ch as u32;
    if (0x0100..=0x01FF).contains(&code) {
        Some((code - 0x0100) as u8)
    } else {
        None
    }
}

fn print_usage() {
    println!("Usage:");
    println!("  bytefall                 # Launch the interactive wizard");
    println!("  bytefall -length <n>     # Run Bytefall pad generation headlessly (no wizard)");
}

fn generate_pad_headless(mut rng: OsRng, target_len: usize) -> std::io::Result<()> {
    let mut pad_writer = PadWriter::new(DEFAULT_PAD_PATH)?;
    let mut chunk = vec![0u8; 4096];
    let mut written = 0usize;

    while written < target_len {
        let remaining = target_len - written;
        let chunk_size = remaining.min(chunk.len());
        rng.fill_bytes(&mut chunk[..chunk_size]);

        for byte in &chunk[..chunk_size] {
            pad_writer.push_byte(*byte)?;
        }

        written += chunk_size;
    }

    pad_writer.finish()?;
    println!("Generated {target_len} bytes to {}", DEFAULT_PAD_PATH);
    Ok(())
}

fn run_animation(mut rng: OsRng, target_len: Option<usize>) -> std::io::Result<()> {
    if let Some(length) = target_len {
        return generate_pad_headless(rng, length);
    }

    let mut particles: Vec<Particle> = Vec::new();
    let mut lane_index: usize = 0;
    let lane_count: usize = 24;
    let stream_center_x = WIDTH as f32 / 2.0;
    let swirl_center_y = 6.0;
    let pad_rows: usize = HEIGHT.saturating_sub(2);
    let mut pad_wall = PadWall::new(pad_rows);
    let mut pad_writer = PadWriter::new(DEFAULT_PAD_PATH)?;
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

struct EncryptionResult {
    output_path: PathBuf,
    pad_path: PathBuf,
}

struct PadEncryptResult {
    output_path: PathBuf,
    start: usize,
    end: usize,
}

struct PadDecryptResult {
    output_path: PathBuf,
}

fn prompt_line(message: &str) -> io::Result<String> {
    print!("{message}");
    stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    Ok(input.trim().to_string())
}

fn prompt_multiline(message: &str) -> io::Result<String> {
    println!("{message}");
    println!("(Enter a blank line to finish.)");
    let mut lines = Vec::new();
    loop {
        let mut line = String::new();
        let read = io::stdin().read_line(&mut line)?;
        if read == 0 {
            break;
        }
        let trimmed = line.trim_end_matches(['\n', '\r']);
        if trimmed.is_empty() {
            break;
        }
        lines.push(trimmed.to_string());
    }
    Ok(lines.join("\n"))
}

fn prompt_pad_length() -> io::Result<Option<usize>> {
    loop {
        println!("How long do you want the pad to be?");
        println!("  1) Infinite");
        println!("  2) Enter a byte length");
        let choice = prompt_line("> ")?;

        match choice.as_str() {
            "1" => return Ok(None),
            "2" => {
                let length_str = prompt_line("Enter pad length (bytes): ")?;
                match length_str.parse::<usize>() {
                    Ok(value) => return Ok(Some(value)),
                    Err(_) => println!("Please enter a valid number."),
                }
            }
            _ => println!("Please choose 1 or 2."),
        }
    }
}

fn encrypt_file(file_path: &str, rng: &mut OsRng) -> io::Result<EncryptionResult> {
    let input_path = Path::new(file_path);
    let mut input = Vec::new();
    File::open(input_path)?.read_to_end(&mut input)?;

    let mut pad = vec![0u8; input.len()];
    rng.fill_bytes(&mut pad);

    let parent = input_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("bytefall");

    let output_path = parent.join(format!("{stem}.glyph"));
    let pad_path = parent.join(format!("{stem}.glyphkey.bin"));

    let mut pad_writer = PadWriter::new(pad_path.to_string_lossy().as_ref())?;

    for byte in &pad {
        pad_writer.push_byte(*byte)?;
    }
    pad_writer.finish()?;

    let mut encrypted = Vec::with_capacity(input.len());
    for (idx, byte) in input.iter().enumerate() {
        let pad_byte = pad[idx];
        encrypted.push(byte ^ pad_byte);
    }

    File::create(&output_path)?.write_all(&encrypted)?;

    Ok(EncryptionResult {
        output_path,
        pad_path,
    })
}

fn read_pad(path: &str) -> io::Result<Vec<u8>> {
    let mut pad = Vec::new();
    File::open(path)?.read_to_end(&mut pad)?;
    Ok(pad)
}

fn read_pad_index_path(pad_path: &Path) -> PathBuf {
    let mut idx = pad_path.to_path_buf();
    let new_ext = match pad_path.extension().and_then(|e| e.to_str()) {
        Some(ext) => format!("{ext}.idx"),
        None => "idx".to_string(),
    };
    idx.set_extension(new_ext);
    idx
}

struct PadIndexGuard {
    file: File,
}

fn lock_pad_index(pad_path: &Path) -> io::Result<PadIndexGuard> {
    let idx_path = read_pad_index_path(pad_path);
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .open(idx_path)?;
    file.lock_exclusive()?;
    Ok(PadIndexGuard { file })
}

impl PadIndexGuard {
    fn read(&mut self) -> io::Result<usize> {
        self.file.seek(SeekFrom::Start(0))?;
        let mut buf = String::new();
        self.file.read_to_string(&mut buf)?;
        if buf.trim().is_empty() {
            return Ok(0);
        }
        match buf.trim().parse::<usize>() {
            Ok(v) => Ok(v),
            Err(_) => Ok(0),
        }
    }

    fn write(&mut self, value: usize) -> io::Result<()> {
        self.file.set_len(0)?;
        self.file.seek(SeekFrom::Start(0))?;
        self.file.write_all(value.to_string().as_bytes())?;
        self.file.sync_all()
    }
}

fn read_pad_slice(path: &str, start: usize, end: usize) -> io::Result<Vec<u8>> {
    if start > end {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "Pad range has invalid offsets",
        ));
    }

    let mut file = File::open(path)?;
    let metadata = file.metadata()?;
    let file_len: u64 = metadata.len();
    let end_u64 = end as u64;
    let start_u64 = start as u64;
    if end_u64 > file_len {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Pad range is out of bounds",
        ));
    }

    file.seek(SeekFrom::Start(start_u64))?;
    let mut buf = vec![0u8; end - start];
    file.read_exact(&mut buf)?;
    Ok(buf)
}

fn pad_length_bytes(path: &str) -> io::Result<usize> {
    let file = File::open(path)?;
    let len = file.metadata()?.len();
    usize::try_from(len).map_err(|_| {
        io::Error::new(
            io::ErrorKind::InvalidData,
            "Pad file is too large to fit in memory on this platform",
        )
    })
}

fn resolve_glyph_pad_path(prompt: &str) -> io::Result<String> {
    let default = Path::new(DEFAULT_PAD_PATH);
    if default.exists() {
        return Ok(String::from(DEFAULT_PAD_PATH));
    }

    loop {
        let path = prompt_line(prompt)?;
        if path.is_empty() {
            println!("Please enter a valid pad path.");
            continue;
        }
        return Ok(path);
    }
}

fn resolve_glyph_key_path(enc_path: &str, prompt: &str) -> io::Result<String> {
    let enc_path = Path::new(enc_path);
    let parent = enc_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = enc_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("bytefall");
    let candidate = parent.join(format!("{stem}.glyphkey.bin"));
    if candidate.exists() {
        return Ok(candidate.to_string_lossy().to_string());
    }

    loop {
        let path = prompt_line(prompt)?;
        if path.is_empty() {
            println!("Please enter a valid key path.");
            continue;
        }
        return Ok(path);
    }
}

fn decrypt_file(enc_path: &str, pad_path: &str) -> io::Result<PathBuf> {
    let input_path = Path::new(enc_path);
    let mut input = Vec::new();
    File::open(input_path)?.read_to_end(&mut input)?;

    let pad = read_pad(pad_path)?;
    if pad.len() != input.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Pad length does not match encrypted file length",
        ));
    }

    let mut decrypted = Vec::with_capacity(input.len());
    for (idx, byte) in input.iter().enumerate() {
        decrypted.push(byte ^ pad[idx]);
    }

    let parent = input_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("bytefall");
    let output_path = parent.join(format!("{stem}.dec"));
    File::create(&output_path)?.write_all(&decrypted)?;
    Ok(output_path)
}

const PAD_HEADER_PREFIX: &str = "BYTEFALL-PAD-OFFSET:";
const PAD_HASH_LEN: usize = 32; // SHA-256 output bytes

fn parse_pad_header(line: &str) -> io::Result<(usize, usize)> {
    let trimmed = line.trim();
    if !trimmed.starts_with(PAD_HEADER_PREFIX) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Missing pad offset header",
        ));
    }
    let rest = trimmed[PAD_HEADER_PREFIX.len()..].trim();
    let mut parts = rest.split('-');
    let start = parts
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "Missing start offset"))?
        .parse::<usize>()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "Invalid start offset"))?;
    let end = parts
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "Missing end offset"))?
        .parse::<usize>()
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "Invalid end offset"))?;
    if start >= end {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Pad header has invalid range",
        ));
    }
    Ok((start, end))
}

fn const_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

fn glyphs_to_bytes(input: &str) -> io::Result<Vec<u8>> {
    let mut out = Vec::new();
    for ch in input.chars() {
        if ch == '\n' || ch == '\r' || ch.is_whitespace() {
            continue;
        }
        if let Some(byte) = byte_from_glyph(ch) {
            out.push(byte);
        }
    }
    Ok(out)
}

fn bytes_to_glyph_lines(bytes: &[u8]) -> String {
    let mut out = String::new();
    for (idx, byte) in bytes.iter().enumerate() {
        out.push(glyph_from_byte(*byte));
        if (idx + 1) % 64 == 0 {
            out.push('\n');
        }
    }
    if !out.ends_with('\n') {
        out.push('\n');
    }
    out
}

fn pad_balance(pad_path: &str) -> io::Result<(usize, usize)> {
    let total = pad_length_bytes(pad_path)?;
    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let used = idx_guard.read()?;
    if used > total {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Pad index exceeds pad length",
        ));
    }
    Ok((used, total))
}

fn pad_encrypt(file_path: &str, pad_path: &str) -> io::Result<PadEncryptResult> {
    let input_path = Path::new(file_path);
    let mut input = Vec::new();
    File::open(input_path)?.read_to_end(&mut input)?;

    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let start = idx_guard.read()?;
    let cipher_end = start + input.len();
    let hash_key_start = cipher_end;
    let hash_key_end = hash_key_start + PAD_HASH_LEN;
    let header = format!("{PAD_HEADER_PREFIX} {start}-{cipher_end}\n");
    let pad_slice = read_pad_slice(pad_path, start, cipher_end)?;
    let hash_key = read_pad_slice(pad_path, hash_key_start, hash_key_end)?;

    let mut encrypted = Vec::with_capacity(input.len());
    for (idx, byte) in input.iter().enumerate() {
        encrypted.push(byte ^ pad_slice[idx]);
    }

    let mut hasher = Sha256::new();
    hasher.update(header.as_bytes());
    hasher.update(&encrypted);
    let hash = hasher.finalize();
    let mut hash_encrypted = Vec::with_capacity(PAD_HASH_LEN);
    for (idx, byte) in hash.as_slice().iter().enumerate() {
        hash_encrypted.push(byte ^ hash_key[idx]);
    }

    let parent = input_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("bytefall");
    let output_path = parent.join(format!("{stem}.glyphs"));

    let mut file = File::create(&output_path)?;
    file.write_all(header.as_bytes())?;
    file.write_all(&hash_encrypted)?;
    file.write_all(&encrypted)?;

    idx_guard.write(hash_key_end)?;

    Ok(PadEncryptResult {
        output_path,
        start,
        end: hash_key_end,
    })
}

fn pad_decrypt(enc_path: &str, pad_path: &str) -> io::Result<PadDecryptResult> {
    let input_path = Path::new(enc_path);
    let mut reader = BufReader::new(File::open(input_path)?);
    let mut header_line = String::new();
    reader.read_line(&mut header_line)?;
    let (start, end) = parse_pad_header(&header_line)?;

    let mut hash_encrypted = vec![0u8; PAD_HASH_LEN];
    reader.read_exact(&mut hash_encrypted)?;

    let mut ciphertext = Vec::new();
    reader.read_to_end(&mut ciphertext)?;

    if ciphertext.len() != end.saturating_sub(start) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Ciphertext length does not match pad range",
        ));
    }

    let pad_slice = read_pad_slice(pad_path, start, end)?;
    let hash_key = read_pad_slice(pad_path, end, end + PAD_HASH_LEN)?;
    let mut decrypted = Vec::with_capacity(ciphertext.len());
    for (idx, byte) in ciphertext.iter().enumerate() {
        decrypted.push(byte ^ pad_slice[idx]);
    }

    let mut hasher = Sha256::new();
    hasher.update(header_line.as_bytes());
    hasher.update(&ciphertext);
    let hash = hasher.finalize();
    let mut hash_decrypted = Vec::with_capacity(PAD_HASH_LEN);
    for (idx, byte) in hash_encrypted.iter().enumerate() {
        hash_decrypted.push(byte ^ hash_key[idx]);
    }
    if !const_time_eq(hash.as_slice(), &hash_decrypted) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Hash verification failed",
        ));
    }

    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let current_idx = idx_guard.read()?;
    let next_idx = current_idx.max(end + PAD_HASH_LEN);
    idx_guard.write(next_idx)?;

    let parent = input_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("bytefall");
    let output_path = parent.join(format!("{stem}.glyphs.dec"));
    File::create(&output_path)?.write_all(&decrypted)?;

    Ok(PadDecryptResult { output_path })
}

fn pad_message_encrypt(pad_path: &str, plaintext: &str) -> io::Result<(String, usize, usize)> {
    let bytes = plaintext.as_bytes();
    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let start = idx_guard.read()?;
    let end = start + bytes.len();

    let pad_slice = read_pad_slice(pad_path, start, end)?;

    let mut ciphertext = Vec::with_capacity(bytes.len());
    for (idx, byte) in bytes.iter().enumerate() {
        ciphertext.push(byte ^ pad_slice[idx]);
    }

    let mut out = String::new();
    let header = format!("{PAD_HEADER_PREFIX} {start}-{end}\n");
    out.push_str(&header);
    out.push_str(&bytes_to_glyph_lines(&ciphertext));

    idx_guard.write(end)?;

    Ok((out, start, end))
}

fn pad_message_decrypt(pad_path: &str, message: &str) -> io::Result<(String, usize, usize)> {
    let mut lines = message.lines();
    let header_line = lines
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "Missing header line"))?;
    let (start, end) = parse_pad_header(header_line)?;

    let mut ciphertext_glyphs = String::new();
    for line in lines {
        ciphertext_glyphs.push_str(line);
        ciphertext_glyphs.push('\n');
    }

    let ciphertext = glyphs_to_bytes(&ciphertext_glyphs)?;
    if ciphertext.len() != end.saturating_sub(start) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Ciphertext length does not match pad range",
        ));
    }

    let pad_slice = read_pad_slice(pad_path, start, end)?;

    let mut plaintext = Vec::with_capacity(ciphertext.len());
    for (idx, byte) in ciphertext.iter().enumerate() {
        plaintext.push(byte ^ pad_slice[idx]);
    }

    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let current_idx = idx_guard.read()?;
    let next_idx = current_idx.max(end);
    idx_guard.write(next_idx)?;

    let plaintext_str = String::from_utf8(plaintext)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "Message is not valid UTF-8"))?;

    Ok((plaintext_str, start, end))
}

fn run_wizard() -> io::Result<()> {
    fn print_menu() {
        println!("Welcome to Bytefall!");
        println!("How can I help you?");
        println!("  1) Generate pad");
        println!("  2) Quick encrypt");
        println!("  3) Quick decrypt");
        println!("  4) Pad encrypt");
        println!("  5) Pad decrypt");
        println!("  6) Pad message");
        println!("  7) Pad balance");
        println!("  8) Quit");
    }

    loop {
        print_menu();
        let choice = prompt_line("> ")?;

        match choice.as_str() {
            "1" => {
                let pad_length = prompt_pad_length()?;
                if let Err(err) = run_animation(OsRng, pad_length) {
                    println!("Generation failed: {err}");
                }
            }
            "2" => {
                let file_path = prompt_line("Enter the path to the file: ")?;
                if file_path.is_empty() {
                    println!("Please enter a valid file path.");
                    continue;
                }

                let mut rng = OsRng;
                if let Err(err) = encrypt_file(&file_path, &mut rng).map(|result| {
                    println!("Encryption complete.");
                    println!("Encrypted file: {}", result.output_path.display());
                    println!("Pad key: {}", result.pad_path.display());
                }) {
                    println!("Encryption failed: {err}");
                }
            }
            "3" => {
                let enc_path = prompt_line("Enter the path to the glyph-encrypted file: ")?;
                if enc_path.is_empty() {
                    println!("Please enter a valid file path.");
                    continue;
                }
                let pad_path =
                    resolve_glyph_key_path(&enc_path, "Enter the path to the pad key: ")?;

                match decrypt_file(&enc_path, &pad_path) {
                    Ok(output_path) => {
                        println!("Decryption complete.");
                        println!("Decrypted file: {}", output_path.display());
                    }
                    Err(err) => {
                        println!("Decryption failed: {err}");
                    }
                }
            }
            "4" => {
                let pad_path = resolve_glyph_pad_path(
                    &format!("Enter the path to {DEFAULT_PAD_PATH}: "),
                )?;
                let file_path = prompt_line("Enter the path to the file to encrypt: ")?;
                if file_path.is_empty() {
                    println!("Please enter a valid file path.");
                    continue;
                }

                match pad_encrypt(&file_path, &pad_path) {
                    Ok(result) => {
                        println!("Pad encryption complete.");
                        println!("Encrypted file: {}", result.output_path.display());
                        println!(
                            "Pad bytes used: {}-{} (start-end, end exclusive)",
                            result.start, result.end
                        );
                    }
                    Err(err) => {
                        println!("Pad encryption failed: {err}");
                    }
                }
            }
            "5" => {
                let enc_path = prompt_line("Enter the path to the .glyphs encrypted file: ")?;
                if enc_path.is_empty() {
                    println!("Please enter a valid file path.");
                    continue;
                }
                let pad_path = resolve_glyph_pad_path(
                    &format!("Enter the path to {DEFAULT_PAD_PATH}: "),
                )?;

                match pad_decrypt(&enc_path, &pad_path) {
                    Ok(result) => {
                        println!("Pad decryption complete.");
                        println!("Decrypted file: {}", result.output_path.display());
                    }
                    Err(err) => {
                        println!("Pad decryption failed: {err}");
                    }
                }
            }
            "6" => {
                let pad_path = resolve_glyph_pad_path(
                    &format!("Enter the path to {DEFAULT_PAD_PATH}: "),
                )?;
                let mode = prompt_line("Encrypt or decrypt? (e/d): ")?;

                match mode.to_lowercase().as_str() {
                    "e" => {
                        let plaintext = prompt_multiline(
                            "Enter the message to encrypt (glyph output will follow):",
                        )?;
                        if plaintext.is_empty() {
                            println!("Message cannot be empty.");
                            continue;
                        }

                        match pad_message_encrypt(&pad_path, &plaintext) {
                            Ok((message, start, end)) => {
                                println!(
                                    "Pad bytes used: {}-{} (start-end, end exclusive)",
                                    start, end
                                );
                                println!("Encrypted message (copy/paste below):");
                                println!("{}", message);
                            }
                            Err(err) => {
                                println!("Message encryption failed: {err}");
                            }
                        }
                    }
                    "d" => {
                        let pasted = prompt_multiline(
                            "Paste the encrypted glyph message (including headers) and leave a blank line to finish:",
                        )?;
                        if pasted.is_empty() {
                            println!("Message cannot be empty.");
                            continue;
                        }

                        match pad_message_decrypt(&pad_path, &pasted) {
                            Ok((plaintext, start, end)) => {
                                println!(
                                    "Pad bytes consumed: {}-{} (start-end, end exclusive)",
                                    start, end
                                );
                                println!("Decrypted message:");
                                println!("{}", plaintext);
                            }
                            Err(err) => {
                                println!("Message decryption failed: {err}");
                            }
                        }
                    }
                    _ => {
                        println!("Please choose 'e' for encrypt or 'd' for decrypt.");
                    }
                }
            }
            "7" => {
                let pad_path = resolve_glyph_pad_path(
                    &format!("Enter the path to {DEFAULT_PAD_PATH}: "),
                )?;
                match pad_balance(&pad_path) {
                    Ok((used, total)) => {
                        let remaining = total.saturating_sub(used);
                        println!("Pad total bytes: {total}");
                        println!("Pad used bytes: {used}");
                        println!("Pad remaining bytes: {remaining}");
                    }
                    Err(err) => {
                        println!("Unable to read pad balance: {err}");
                    }
                }
            }
            "8" => return Ok(()),
            _ => println!("Please choose 1, 2, 3, 4, 5, 6, 7, or 8."),
        }

        println!("\n----------------------------------------\n");
    }
}

fn main() {
    let mut args = env::args().skip(1).peekable();

    let result = if args.peek().is_none() {
        run_wizard()
    } else {
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

        run_animation(OsRng, pad_length)
    };

    if let Err(err) = result {
        eprintln!("Bytefall error: {err}");
    }
}
