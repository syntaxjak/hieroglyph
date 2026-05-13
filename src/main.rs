use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use fs2::FileExt;
use rand::{rngs::OsRng, RngCore};
use sha2::{Digest, Sha256};
use std::collections::VecDeque;
use std::env;
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use ratatui::text::{Line, Span};

const DEFAULT_PAD_PATH: &str = "pad.bin";
const GLYPH_ENCRYPTED_EXT: &str = "glyph";
const PAD_ENCRYPTED_EXT: &str = "glyphs";
const MESSAGE_PAD_PATH: &str = "message_pad.txt";

struct PadWriter {
    writer: BufWriter<File>,
    pub count: usize,
}

impl PadWriter {
    fn new(path: &str) -> io::Result<Self> {
        let file = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;
        file.lock_exclusive()?;

        Ok(Self {
            writer: BufWriter::new(file),
            count: 0,
        })
    }

    fn push_byte(&mut self, byte: u8) -> io::Result<()> {
        self.writer.write_all(&[byte])?;
        self.count += 1;
        Ok(())
    }

    fn finish(&mut self) -> io::Result<()> {
        self.writer.flush()?;
        self.writer.get_ref().sync_all()?;
        Ok(())
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
    use ratatui::prelude::*;
    use ratatui::widgets::*;

    if target_len.is_none() {
        return generate_pad_headless(rng, 1024 * 1024);
    }

    let target_len = target_len.unwrap_or(0);
    let mut terminal = ratatui::init();
    let mut pad_writer = PadWriter::new(DEFAULT_PAD_PATH)?;
    let mut chunk = vec![0u8; 4096];
    let mut recent = VecDeque::<u8>::new();

    while pad_writer.count < target_len {
        let remaining = target_len - pad_writer.count;
        let chunk_size = remaining.min(chunk.len());
        rng.fill_bytes(&mut chunk[..chunk_size]);
        for byte in &chunk[..chunk_size] {
            pad_writer.push_byte(*byte)?;
            recent.push_back(*byte);
            if recent.len() > 64 {
                recent.pop_front();
            }
        }

        let done = pad_writer.count;
        terminal.draw(|f| {
            let area = f.area();
            let vertical = Layout::vertical([
                Constraint::Length(3),
                Constraint::Length(3),
                Constraint::Min(5),
            ])
            .split(area);

            let title = Paragraph::new("Hieroglyph - Pad Generation")
                .block(Block::bordered().title("Status"));
            f.render_widget(title, vertical[0]);

            let gauge = Gauge::default()
                .block(Block::bordered().title("Progress"))
                .ratio(done as f64 / target_len as f64)
                .label(format!("{done}/{target_len} bytes"));
            f.render_widget(gauge, vertical[1]);

            let glyphs: String = recent.iter().map(|b| glyph_from_byte(*b)).collect();
            let preview = Paragraph::new(glyphs)
                .block(Block::bordered().title("Recent glyph stream"));
            f.render_widget(preview, vertical[2]);
        })?;
    }

    pad_writer.finish()?;
    ratatui::restore();
    println!("Generated {target_len} bytes to {}", DEFAULT_PAD_PATH);
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

fn list_files_with_extension(ext: &str) -> io::Result<Vec<String>> {
    let mut matches = Vec::new();
    for entry in std::fs::read_dir(env::current_dir()?)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_file() && path.extension().and_then(|e| e.to_str()) == Some(ext) {
            matches.push(path.to_string_lossy().to_string());
        }
    }
    matches.sort();
    Ok(matches)
}

fn detect_encryptable_files() -> io::Result<Vec<String>> {
    let mut matches = Vec::new();
    for entry in std::fs::read_dir(env::current_dir()?)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        matches.push(path.to_string_lossy().to_string());
    }
    matches.sort();
    Ok(matches)
}

fn auto_glyph_key_path(enc_path: &str) -> Option<String> {
    let enc_path = Path::new(enc_path);
    let parent = enc_path.parent().unwrap_or_else(|| Path::new("."));
    let stem = enc_path.file_stem()?.to_string_lossy();
    let candidate = parent.join(format!("{stem}.glyphkey.bin"));
    if candidate.exists() {
        return Some(candidate.to_string_lossy().to_string());
    }
    None
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

fn append_message_pad(kind: &str, content: &str) {
    if let Ok(mut file) = OpenOptions::new()
        .create(true)
        .append(true)
        .open(MESSAGE_PAD_PATH)
    {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or(Duration::from_secs(0))
            .as_secs();
        let _ = writeln!(file, "--- {kind} @ {ts} ---");
        let _ = writeln!(file, "{}", content.trim_end());
        let _ = writeln!(file);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum WizardAction {
    GeneratePad,
    QuickEncrypt,
    QuickDecrypt,
    PadEncrypt,
    PadDecrypt,
    PadMessageEncrypt,
    PadMessageDecrypt,
    PadBalance,
    Quit,
}

struct InputField {
    label: String,
    value: String,
    multiline: bool,
}

impl InputField {
    fn new(label: &str, value: String, multiline: bool) -> Self {
        Self {
            label: label.to_string(),
            value,
            multiline,
        }
    }
}

struct PadProgress {
    target_len: usize,
    pad_writer: PadWriter,
    recent: VecDeque<u8>,
    chunk: Vec<u8>,
    done: usize,
    rng: OsRng,
    finished: bool,
    error: Option<String>,
}

impl PadProgress {
    fn new(target_len: usize) -> io::Result<Self> {
        Ok(Self {
            target_len,
            pad_writer: PadWriter::new(DEFAULT_PAD_PATH)?,
            recent: VecDeque::new(),
            chunk: vec![0u8; 4096],
            done: 0,
            rng: OsRng,
            finished: false,
            error: None,
        })
    }

    fn step(&mut self) -> io::Result<()> {
        if self.finished {
            return Ok(());
        }
        if self.done >= self.target_len {
            self.pad_writer.finish()?;
            self.finished = true;
            return Ok(());
        }

        let remaining = self.target_len - self.done;
        let chunk_size = remaining.min(self.chunk.len());
        self.rng.fill_bytes(&mut self.chunk[..chunk_size]);
        for byte in &self.chunk[..chunk_size] {
            self.pad_writer.push_byte(*byte)?;
            self.recent.push_back(*byte);
            if self.recent.len() > 64 {
                self.recent.pop_front();
            }
        }
        self.done += chunk_size;

        if self.done >= self.target_len {
            self.pad_writer.finish()?;
            self.finished = true;
        }
        Ok(())
    }

    fn ratio(&self) -> f64 {
        if self.target_len == 0 {
            0.0
        } else {
            self.done as f64 / self.target_len as f64
        }
    }

    fn glyph_preview(&self) -> String {
        self.recent.iter().map(|b| glyph_from_byte(*b)).collect()
    }
}

struct ActionView {
    action: WizardAction,
    fields: Vec<InputField>,
    selected: usize,
    editing: bool,
    status: Vec<String>,
    busy: bool,
    pad_progress: Option<PadProgress>,
    available: Vec<String>,
    focus: Focus,
    candidate_idx: usize,
    output_panel: Option<(String, String)>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Focus {
    Fields,
    Candidates,
}

enum Screen {
    Menu { selected: usize },
    Action(ActionView),
}

struct App {
    screen: Screen,
    last_tick: Instant,
}

fn push_status(view: &mut ActionView, msg: impl Into<String>) {
    view.status.push(msg.into());
    if view.status.len() > 8 {
        let overflow = view.status.len() - 8;
        view.status.drain(0..overflow);
    }
}

fn action_label(action: WizardAction) -> &'static str {
    match action {
        WizardAction::GeneratePad => "Generate pad",
        WizardAction::QuickEncrypt => "Quick encrypt",
        WizardAction::QuickDecrypt => "Quick decrypt",
        WizardAction::PadEncrypt => "Pad encrypt",
        WizardAction::PadDecrypt => "Pad decrypt",
        WizardAction::PadMessageEncrypt => "Pad message (encrypt)",
        WizardAction::PadMessageDecrypt => "Pad message (decrypt)",
        WizardAction::PadBalance => "Pad balance",
        WizardAction::Quit => "Quit",
    }
}

fn build_action_view(action: WizardAction) -> io::Result<ActionView> {
    let mut status = Vec::new();
    status.push(format!("{}", action_label(action)));

    let (fields, mut available) = match action {
        WizardAction::GeneratePad => (
            vec![InputField::new(
                "Pad length bytes (blank for 1048576)",
                String::new(),
                false,
            )],
            Vec::new(),
        ),
        WizardAction::QuickEncrypt => {
            let detected = detect_encryptable_files()?;
            let default = detected.first().cloned().unwrap_or_default();
            (
                vec![InputField::new("File to encrypt", default, false)],
                detected,
            )
        }
        WizardAction::QuickDecrypt => {
            let encs = list_files_with_extension(GLYPH_ENCRYPTED_EXT)?;
            let enc_default = encs.first().cloned().unwrap_or_default();
            let key_default = auto_glyph_key_path(&enc_default).unwrap_or_default();
            (
                vec![
                    InputField::new("Encrypted .glyph file", enc_default, false),
                    InputField::new("Pad key (.glyphkey.bin)", key_default, false),
                ],
                encs,
            )
        }
        WizardAction::PadEncrypt => {
            let detected = detect_encryptable_files()?;
            let default_file = detected.first().cloned().unwrap_or_default();
            (
                vec![InputField::new("File to pad-encrypt", default_file, false)],
                detected,
            )
        }
        WizardAction::PadDecrypt => {
            let encs = list_files_with_extension(PAD_ENCRYPTED_EXT)?;
            let enc_default = encs.first().cloned().unwrap_or_default();
            (
                vec![InputField::new("Encrypted .glyphs file", enc_default, false)],
                encs,
            )
        }
        WizardAction::PadMessageEncrypt => (
            vec![InputField::new("Message to encrypt", String::new(), true)],
            Vec::new(),
        ),
        WizardAction::PadMessageDecrypt => (
            vec![InputField::new("Glyph message to decrypt", String::new(), true)],
            Vec::new(),
        ),
        WizardAction::PadBalance => (Vec::new(), Vec::new()),
        WizardAction::Quit => (Vec::new(), Vec::new()),
    };

    available.sort();
    let candidate_idx = 0;
    let focus = if matches!(action, WizardAction::QuickEncrypt | WizardAction::PadEncrypt)
        && !available.is_empty()
    {
        Focus::Candidates
    } else {
        Focus::Fields
    };

    let editing_default = focus == Focus::Fields
        && !fields.is_empty()
        && !matches!(action, WizardAction::GeneratePad | WizardAction::Quit);

    Ok(ActionView {
        action,
        fields,
        selected: 0,
        editing: editing_default,
        status,
        busy: false,
        pad_progress: None,
        available,
        focus,
        candidate_idx,
        output_panel: None,
    })
}

fn draw_menu(f: &mut ratatui::Frame, selected: usize) {
    use ratatui::layout::*;
    use ratatui::widgets::*;

    let area = f.area();
    let chunks = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(5),
        Constraint::Length(2),
    ])
    .split(area);

    let title = Paragraph::new("Hieroglyph - TUI wizard")
        .alignment(Alignment::Center)
        .block(Block::bordered().title("Welcome"));
    f.render_widget(title, chunks[0]);

    let actions = [
        WizardAction::GeneratePad,
        WizardAction::QuickEncrypt,
        WizardAction::QuickDecrypt,
        WizardAction::PadEncrypt,
        WizardAction::PadDecrypt,
        WizardAction::PadMessageEncrypt,
        WizardAction::PadMessageDecrypt,
        WizardAction::PadBalance,
        WizardAction::Quit,
    ];
    let items: Vec<ListItem> = actions
        .iter()
        .enumerate()
        .map(|(idx, action)| {
            let content = format!("{} {}", if idx == selected {"→"} else {" "}, action_label(*action));
            ListItem::new(content)
        })
        .collect();

    let list = List::new(items)
        .block(Block::bordered().title("Choose an action"))
        .highlight_symbol("");
    f.render_widget(list, chunks[1]);

    let footer = Paragraph::new("Use ↑/↓ to move, Enter to select, q to quit.")
        .alignment(Alignment::Center)
        .block(Block::bordered());
    f.render_widget(footer, chunks[2]);
}

fn draw_action(f: &mut ratatui::Frame, view: &ActionView) {
    use ratatui::layout::*;
    use ratatui::style::*;
    use ratatui::widgets::*;

    let area = f.area();
    let outer = Layout::vertical([
        Constraint::Length(3),
        Constraint::Min(7),
        Constraint::Length(3),
    ])
    .split(area);

    let header_text = format!(
        "{}{}",
        action_label(view.action),
        if view.busy { " (running...)" } else { "" }
    );
    let header = Paragraph::new(header_text)
        .alignment(Alignment::Center)
        .block(Block::bordered().title("Action"));
    f.render_widget(header, outer[0]);

    let mid = Layout::horizontal([
        Constraint::Percentage(55),
        Constraint::Percentage(45),
    ])
    .split(outer[1]);

    let show_candidates = !view.available.is_empty()
        && matches!(view.action, WizardAction::QuickEncrypt | WizardAction::PadEncrypt);

    let left_chunks = if show_candidates {
        Layout::vertical([
            Constraint::Min(5),
            Constraint::Length((view.available.len().saturating_mul(1) + 2) as u16),
        ])
        .split(mid[0])
    } else {
        Layout::vertical([Constraint::Min(5)]).split(mid[0])
    };

    let mut items = Vec::new();
    for (idx, field) in view.fields.iter().enumerate() {
        let mut lines = Vec::new();
        lines.push(Line::from(vec![Span::styled(
            format!("{}:", field.label),
            Style::default().fg(Color::Cyan),
        )]));
        if field.value.is_empty() {
            lines.push(Line::from(Span::styled(
                "<empty>",
                Style::default().fg(Color::DarkGray),
            )));
        } else {
            for val_line in field.value.lines() {
                lines.push(Line::from(val_line.to_string()));
            }
        }

        let mut item = ListItem::new(lines);
        if view.focus == Focus::Fields && idx == view.selected {
            let style = if view.editing {
                Style::default().fg(Color::Yellow)
            } else {
                Style::default().fg(Color::LightGreen)
            };
            item = item.style(style);
        }
        items.push(item);
    }

    let list = List::new(items)
        .block(Block::bordered().title("Fields"))
        .highlight_symbol("");
    f.render_widget(list, left_chunks[0]);

    if show_candidates {
        let mut list_items = Vec::new();
        for (idx, path) in view.available.iter().enumerate() {
            let prefix = if view.focus == Focus::Candidates && view.candidate_idx == idx {
                "→ "
            } else {
                "  "
            };
            list_items.push(ListItem::new(format!("{}{}", prefix, path)));
        }
        let list = List::new(list_items)
            .block(Block::bordered().title("Files"))
            .highlight_symbol("");
        f.render_widget(list, left_chunks[1]);
    }

    if let Some(progress) = &view.pad_progress {
        let gauge_area = Layout::vertical([
            Constraint::Length(3),
            Constraint::Min(3),
        ])
        .split(mid[1]);
        let gauge = Gauge::default()
            .block(Block::bordered().title("Pad generation"))
            .ratio(progress.ratio())
            .label(format!("{}/{} bytes", progress.done, progress.target_len));
        f.render_widget(gauge, gauge_area[0]);

        let preview = Paragraph::new(progress.glyph_preview())
            .block(Block::bordered().title("Recent glyph stream"));
        f.render_widget(preview, gauge_area[1]);
    } else {
        let show_extra = matches!(view.action, WizardAction::QuickDecrypt | WizardAction::PadDecrypt);
        let has_output = view.output_panel.is_some();

        let mut vertical = Vec::new();
        if has_output {
            vertical.push(Constraint::Length(9));
        }
        if show_extra {
            vertical.push(Constraint::Length(7));
        }
        vertical.push(Constraint::Min(5));

        let cols = Layout::vertical(vertical).split(mid[1]);
        let mut col_idx = 0;

        if let Some((title, content)) = &view.output_panel {
            let output = Paragraph::new(content.clone())
                .block(Block::bordered().title(title.as_str()))
                .wrap(Wrap { trim: false });
            f.render_widget(output, cols[col_idx]);
            col_idx += 1;
        }

        if show_extra {
            let frames = ["⏳", "✦", "✸", "✧", "✺", "✹"];
            let now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or(Duration::from_secs(0));
            let idx = ((now.as_millis() / 150) as usize) % frames.len();
            let glyph_seed = (now.subsec_nanos() as u64).to_le_bytes();
            let mut preview = String::new();
            for b in glyph_seed {
                preview.push(glyph_from_byte(b));
            }
            let extra = Paragraph::new(format!(
                "Decrypt lounge {}\nGlyph drift: {}",
                frames[idx], preview
            ))
            .block(Block::bordered().title("Decryption vibes"));
            f.render_widget(extra, cols[col_idx]);
            col_idx += 1;
        }

        let status_text: String = if view.status.is_empty() {
            "Status messages will appear here.".to_string()
        } else {
            view.status.join("\n")
        };
        let status = Paragraph::new(status_text)
            .block(Block::bordered().title("Status"))
            .wrap(Wrap { trim: true });
        f.render_widget(status, cols[col_idx]);
    }

    let instructions = if view.editing {
        "Esc: stop editing | Backspace: delete | Enter: newline (multiline) / finish edit (single) | Tab/Shift+Tab: move"
    } else if matches!(view.action, WizardAction::QuickEncrypt | WizardAction::PadEncrypt)
        && !view.available.is_empty()
    {
        "Enter: run | e: edit | Tab/Shift+Tab or ↑/↓: move | ↓ from last field to files, ↑ from files to fields | Enter on file: fill"
    } else {
        "Enter: run action | e: edit field | Tab/Shift+Tab or ↑/↓: move | Esc: back to menu"
    };
    let footer = Paragraph::new(instructions)
        .alignment(Alignment::Center)
        .block(Block::bordered());
    f.render_widget(footer, outer[2]);
}

fn handle_char_input(field: &mut InputField, key: KeyCode, modifiers: KeyModifiers) {
    match key {
        KeyCode::Char(c) => {
            if modifiers.contains(KeyModifiers::CONTROL) {
                return;
            }
            field.value.push(c);
        }
        KeyCode::Backspace => {
            field.value.pop();
        }
        KeyCode::Enter => {
            if field.multiline {
                field.value.push('\n');
            }
        }
        KeyCode::Tab | KeyCode::BackTab | KeyCode::Esc => {}
        _ => {}
    }
}

fn run_action_now(view: &mut ActionView) {
    if view.busy {
        return;
    }

    match view.action {
        WizardAction::GeneratePad => {
            let length_str = view.fields[0].value.trim();
            let target_len = if length_str.is_empty() {
                1_048_576usize
            } else {
                match length_str.parse::<usize>() {
                    Ok(v) if v > 0 => v,
                    _ => {
                        push_status(view, "Please enter a positive number for pad length.");
                        return;
                    }
                }
            };

            match PadProgress::new(target_len) {
                Ok(progress) => {
                    view.pad_progress = Some(progress);
                    view.busy = true;
                    push_status(view, format!("Generating {} bytes to {}", target_len, DEFAULT_PAD_PATH));
                }
                Err(err) => push_status(view, format!("Unable to start generation: {err}")),
            }
        }
        WizardAction::QuickEncrypt => {
            let file = view.fields[0].value.trim();
            if file.is_empty() {
                push_status(view, "Please provide a file to encrypt.");
                return;
            }
            view.busy = true;
            match encrypt_file(file, &mut OsRng) {
                Ok(result) => {
                    push_status(view, format!("Encrypted file: {}", result.output_path.display()));
                    push_status(view, format!("Pad key: {}", result.pad_path.display()));
                }
                Err(err) => push_status(view, format!("Encryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::QuickDecrypt => {
            let enc = view.fields[0].value.trim().to_string();
            let mut key = view.fields[1].value.trim().to_string();
            if enc.is_empty() {
                push_status(view, "Please provide an encrypted .glyph file.");
                return;
            }
            if key.is_empty() {
                if let Some(auto) = auto_glyph_key_path(&enc) {
                    key = auto;
                    view.fields[1].value = key.clone();
                    push_status(view, "Auto-selected matching pad key.");
                } else {
                    push_status(view, "Please provide the matching pad key file.");
                    return;
                }
            }
            view.busy = true;
            match decrypt_file(&enc, &key) {
                Ok(path) => push_status(view, format!("Decrypted to {}", path.display())),
                Err(err) => push_status(view, format!("Decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadEncrypt => {
            let pad = DEFAULT_PAD_PATH;
            let file = view.fields[0].value.trim();
            if file.is_empty() {
                push_status(view, "Provide a file path.");
                return;
            }
            view.busy = true;
            match pad_encrypt(file, pad) {
                Ok(result) => {
                    push_status(view, format!("Encrypted file: {}", result.output_path.display()));
                    push_status(view, format!(
                        "Pad bytes used: {}-{} (end exclusive)",
                        result.start, result.end
                    ));
                }
                Err(err) => push_status(view, format!("Pad encryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadDecrypt => {
            let pad = DEFAULT_PAD_PATH;
            let enc = view.fields[0].value.trim();
            if enc.is_empty() {
                push_status(view, "Provide the encrypted file path.");
                return;
            }
            view.busy = true;
            match pad_decrypt(enc, pad) {
                Ok(result) => push_status(view, format!("Decrypted to {}", result.output_path.display())),
                Err(err) => push_status(view, format!("Pad decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadMessageEncrypt => {
            let pad = DEFAULT_PAD_PATH;
            let msg = &view.fields[0].value;
            if msg.trim().is_empty() {
                push_status(view, "Provide a message to encrypt.");
                return;
            }
            view.busy = true;
            match pad_message_encrypt(pad, msg) {
                Ok((cipher, start, end)) => {
                    push_status(view, format!("Pad bytes used: {}-{}", start, end));
                    push_status(view, "Encrypted message shown on the right.");
                    view.output_panel = Some(("Encrypted message".to_string(), cipher.clone()));
                    append_message_pad("ENCRYPTED", &cipher);
                    push_status(view, format!("Saved to {MESSAGE_PAD_PATH}"));
                }
                Err(err) => push_status(view, format!("Message encryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadMessageDecrypt => {
            let pad = DEFAULT_PAD_PATH;
            let msg = &view.fields[0].value;
            if msg.trim().is_empty() {
                push_status(view, "Provide the glyph message.");
                return;
            }
            view.busy = true;
            match pad_message_decrypt(pad, msg) {
                Ok((plain, start, end)) => {
                    push_status(view, format!("Pad bytes consumed: {}-{}", start, end));
                    push_status(view, "Decrypted message shown on the right.");
                    view.output_panel = Some(("Decrypted message".to_string(), plain.clone()));
                    append_message_pad("DECRYPTED", &plain);
                    push_status(view, format!("Saved to {MESSAGE_PAD_PATH}"));
                }
                Err(err) => push_status(view, format!("Message decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadBalance => {
            let pad = DEFAULT_PAD_PATH;
            view.busy = true;
            match pad_balance(pad) {
                Ok((used, total)) => {
                    let remaining = total.saturating_sub(used);
                    push_status(view, format!("Pad total: {} bytes", total));
                    push_status(view, format!("Used: {} bytes", used));
                    push_status(view, format!("Remaining: {} bytes", remaining));
                }
                Err(err) => push_status(view, format!("Unable to read pad balance: {err}")),
            }
            view.busy = false;
        }
        WizardAction::Quit => {}
    }
}

fn run_wizard() -> io::Result<()> {
    use ratatui::Terminal;

    let mut terminal: Terminal<_> = ratatui::init();
    let mut app = App {
        screen: Screen::Menu { selected: 0 },
        last_tick: Instant::now(),
    };

    let tick_rate = Duration::from_millis(30);

    loop {
        terminal.draw(|f| match &app.screen {
            Screen::Menu { selected } => draw_menu(f, *selected),
            Screen::Action(view) => draw_action(f, view),
        })?;

        if let Screen::Action(view) = &mut app.screen {
            let mut fail_message: Option<String> = None;
            let mut complete: Option<usize> = None;
            let mut clear_progress = false;

            if let Some(progress) = &mut view.pad_progress {
                if progress.error.is_none() && !progress.finished {
                    if let Err(err) = progress.step() {
                        progress.error = Some(err.to_string());
                        fail_message = Some(err.to_string());
                        view.busy = false;
                        clear_progress = true;
                    }
                }
                if progress.finished {
                    complete = Some(progress.done);
                    clear_progress = true;
                }
            }

            if clear_progress {
                view.pad_progress = None;
            }

            if let Some(done) = complete {
                view.busy = false;
                push_status(view, format!("Generated {} bytes to {}", done, DEFAULT_PAD_PATH));
            }
            if let Some(err) = fail_message {
                push_status(view, format!("Generation failed: {}", err));
            }
        }

        let timeout = tick_rate.saturating_sub(app.last_tick.elapsed());
        if event::poll(timeout)? {
            app.last_tick = Instant::now();
            if let Event::Key(key) = event::read()? {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                match &mut app.screen {
                    Screen::Menu { selected } => match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => {
                            ratatui::restore();
                            return Ok(());
                        }
                        KeyCode::Up => {
                            if *selected > 0 {
                                *selected -= 1;
                            }
                        }
                        KeyCode::Down => {
                            if *selected < 8 {
                                *selected += 1;
                            }
                        }
                        KeyCode::Enter => {
                            let actions = [
                                WizardAction::GeneratePad,
                                WizardAction::QuickEncrypt,
                                WizardAction::QuickDecrypt,
                                WizardAction::PadEncrypt,
                                WizardAction::PadDecrypt,
                                WizardAction::PadMessageEncrypt,
                                WizardAction::PadMessageDecrypt,
                                WizardAction::PadBalance,
                                WizardAction::Quit,
                            ];
                            let chosen = actions[*selected];
                            if chosen == WizardAction::Quit {
                                ratatui::restore();
                                return Ok(());
                            }
                            match build_action_view(chosen) {
                                Ok(view) => app.screen = Screen::Action(view),
                                Err(err) => {
                                    ratatui::restore();
                                    return Err(err);
                                }
                            }
                        }
                        _ => {}
                    },
                    Screen::Action(view) => {
                        let show_candidates = !view.available.is_empty()
                            && !matches!(view.action, WizardAction::QuickDecrypt | WizardAction::PadDecrypt);

                        if let Some(progress) = &view.pad_progress {
                            if !progress.finished {
                                if key.code == KeyCode::Esc {
                                    app.screen = Screen::Menu { selected: 0 };
                                }
                                continue;
                            }
                        }

                        if view.busy && view.pad_progress.is_none() {
                            if key.code == KeyCode::Esc {
                                app.screen = Screen::Menu { selected: 0 };
                            }
                            continue;
                        }

                        if view.editing {
                            match key.code {
                                KeyCode::Esc => view.editing = false,
                                KeyCode::Tab => {
                                    view.selected = (view.selected + 1) % view.fields.len();
                                }
                                KeyCode::BackTab => {
                                    if view.selected == 0 {
                                        view.selected = view.fields.len() - 1;
                                    } else {
                                        view.selected -= 1;
                                    }
                                }
                                KeyCode::Enter if !view.fields[view.selected].multiline => {
                                    view.editing = false;
                                }
                                _ => {
                                    let field = &mut view.fields[view.selected];
                                    handle_char_input(field, key.code, key.modifiers);
                                }
                            }
                        } else {
                            match key.code {
                                KeyCode::Esc => app.screen = Screen::Menu { selected: 0 },
                                KeyCode::Char('e') => view.editing = true,
                                KeyCode::Tab => {
                                    if !view.fields.is_empty() {
                                        view.focus = Focus::Fields;
                                        view.selected = (view.selected + 1) % view.fields.len();
                                    }
                                }
                                KeyCode::BackTab => {
                                    if !view.fields.is_empty() {
                                        view.focus = Focus::Fields;
                                        if view.selected == 0 {
                                            view.selected = view.fields.len() - 1;
                                        } else {
                                            view.selected -= 1;
                                        }
                                    }
                                }
                                KeyCode::Right => {
                                    if show_candidates {
                                        view.focus = Focus::Candidates;
                                        if view.candidate_idx >= view.available.len() {
                                            view.candidate_idx = 0;
                                        }
                                    }
                                }
                                KeyCode::Left => {
                                    view.focus = Focus::Fields;
                                }
                                KeyCode::Up => {
                                    if view.focus == Focus::Candidates && show_candidates {
                                        if view.candidate_idx == 0 {
                                            view.candidate_idx = view.available.len() - 1;
                                        }
                                        view.focus = Focus::Fields;
                                        if !view.fields.is_empty() {
                                            view.selected = view.fields.len().saturating_sub(1);
                                        }
                                    } else if !view.fields.is_empty() {
                                        if view.selected == 0 {
                                            view.selected = view.fields.len() - 1;
                                        } else {
                                            view.selected -= 1;
                                        }
                                    }
                                }
                                KeyCode::Down => {
                                    if view.focus == Focus::Candidates && show_candidates {
                                        view.candidate_idx = (view.candidate_idx + 1) % view.available.len();
                                    } else if show_candidates && !view.fields.is_empty() && view.selected == view.fields.len() - 1 {
                                        view.focus = Focus::Candidates;
                                        if view.candidate_idx >= view.available.len() {
                                            view.candidate_idx = 0;
                                        }
                                    } else if !view.fields.is_empty() {
                                        view.selected = (view.selected + 1) % view.fields.len();
                                    }
                                }
                                KeyCode::Enter => {
                                    if view.focus == Focus::Candidates && show_candidates {
                                        let choice = view.available[view.candidate_idx].clone();
                                        match view.action {
                                            WizardAction::QuickEncrypt => {
                                                view.fields[0].value = choice;
                                            }
                                            WizardAction::PadEncrypt => {
                                                if !view.fields.is_empty() {
                                                    view.fields[0].value = choice;
                                                }
                                            }
                                            _ => {}
                                        }
                                        view.focus = Focus::Fields;
                                        view.selected = 0;
                                        view.editing = false;
                                    } else {
                                        run_action_now(view);
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                }
            }
        }
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
