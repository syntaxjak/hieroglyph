use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use fs2::FileExt;
use rand::{rngs::OsRng, RngCore};
use sha2::{Digest, Sha256};
use std::env;
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
mod interface;
use interface::{draw_action, draw_menu, handle_action_event};

const DEFAULT_PAD_PATH: &str = "pad.bin";
const MESSAGE_PAD_PATH: &str = "message_pad.txt";

struct PadWriter {
    writer: BufWriter<File>,
    pub count: usize,
}

impl PadWriter {
    fn new(path: &str) -> io::Result<Self> {
        if read_pad_index_path(Path::new(path)).exists() {
            return Err(io::Error::new(io::ErrorKind::AlreadyExists,
                "A pad index already exists for this path. Choose a new pad name; do not reset an old index."));
        }
        let mut options = OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let file = options.open(path).map_err(|err| {
            if err.kind() == io::ErrorKind::AlreadyExists {
                io::Error::new(err.kind(), "Pad already exists. Choose a new path to keep the existing pad safe.")
            } else {
                err
            }
        })?;
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
    println!("  hieroglyph                                  # Open the wizard");
    println!("  hieroglyph --length <size> [--pad <path>]     # Create a new pad without the wizard");
    println!("Sizes: 1024, 64KiB, or \"10 MiB\". Default pad path: {DEFAULT_PAD_PATH}");
    println!("Existing pads are never overwritten. Use separate pads for each sending direction.");
}

fn parse_pad_size(input: &str) -> io::Result<usize> {
    let input = input.trim();
    let digits = input.bytes().take_while(u8::is_ascii_digit).count();
    let amount = input[..digits].parse::<usize>().ok();
    let multiplier = match input[digits..].trim().to_ascii_lowercase().as_str() {
        "" | "b" => 1,
        "kib" => 1024,
        "mib" => 1024 * 1024,
        "gib" => 1024 * 1024 * 1024,
        _ => 0,
    };
    amount.and_then(|n| n.checked_mul(multiplier)).filter(|n| *n > 0)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput,
            "Enter a positive whole-byte size, for example 1024, 64 KiB, or 10 MiB (within platform limits)."))
}

fn format_bytes(bytes: usize) -> String {
    for (unit, factor) in [("GiB", 1024 * 1024 * 1024), ("MiB", 1024 * 1024), ("KiB", 1024)] {
        if bytes >= factor {
            return format!("{:.1} {unit} ({bytes} bytes)", bytes as f64 / factor as f64);
        }
    }
    format!("{bytes} bytes")
}

fn generate_pad_headless(mut rng: OsRng, target_len: usize, path: &str) -> std::io::Result<()> {
    let mut pad_writer = PadWriter::new(path)?;
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
    println!("Created {} at {path}. Share securely; never reuse pad bytes.", format_bytes(target_len));
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

fn checked_pad_end(pad_path: &str, start: usize, needed: usize) -> io::Result<usize> {
    let total = pad_length_bytes(pad_path)?;
    let remaining = total.checked_sub(start).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidData, "Pad index exceeds pad length; do not reset it.")
    })?;
    if needed > remaining {
        return Err(io::Error::new(io::ErrorKind::UnexpectedEof, format!(
            "Not enough unused pad bytes: need {needed}, have {remaining}. Create and securely share a new pad; never reset the index.")));
    }
    Ok(start + needed)
}

fn pad_encrypt(file_path: &str, pad_path: &str) -> io::Result<PadEncryptResult> {
    let input_path = Path::new(file_path);
    let mut input = Vec::new();
    File::open(input_path)?.read_to_end(&mut input)?;

    let mut idx_guard = lock_pad_index(Path::new(pad_path))?;
    let start = idx_guard.read()?;
    let needed = input.len().checked_add(PAD_HASH_LEN).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidInput, "Input is too large")
    })?;
    let hash_key_end = checked_pad_end(pad_path, start, needed)?;
    let cipher_end = hash_key_end - PAD_HASH_LEN;
    let hash_key_start = cipher_end;
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
    let end = checked_pad_end(pad_path, start, bytes.len())?;

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

const WIZARD_ACTIONS: &[WizardAction] = &[
    WizardAction::GeneratePad,
    WizardAction::PadBalance,
    WizardAction::PadEncrypt,
    WizardAction::PadDecrypt,
    WizardAction::PadMessageEncrypt,
    WizardAction::PadMessageDecrypt,
    WizardAction::QuickEncrypt,
    WizardAction::QuickDecrypt,
    WizardAction::Quit,
];

struct InputField {
    label: String,
    value: String,
    multiline: bool,
    cursor: usize,
    select_all: bool,
}

impl InputField {
    fn new(label: &str, value: String, multiline: bool) -> Self {
        Self {
            label: label.to_string(),
            cursor: value.len(),
            value,
            multiline,
            select_all: false,
        }
    }
}

struct PadProgress {
    target_len: usize,
    path: String,
    pad_writer: PadWriter,
    chunk: Vec<u8>,
    done: usize,
    rng: OsRng,
    finished: bool,
    error: Option<String>,
}

impl PadProgress {
    fn new(target_len: usize, path: &str) -> io::Result<Self> {
        Ok(Self {
            target_len,
            path: path.to_string(),
            pad_writer: PadWriter::new(path)?,
            chunk: vec![0u8; 256 * 1024],
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

}

struct ActionView {
    action: WizardAction,
    fields: Vec<InputField>,
    selected: usize,
    status: Vec<String>,
    busy: bool,
    pad_progress: Option<PadProgress>,
    available: Vec<String>,
    focus: Focus,
    candidate_idx: usize,
    output_panel: Option<(String, String)>,
    status_kind: StatusKind,
    preset_idx: usize,
    browse_field: usize,
    picker_dir: PathBuf,
    picker_error: Option<String>,
    output_scroll: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Focus {
    Fields,
    Presets,
    Browse,
    Candidates,
    Primary,
    Back,
    Output,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StatusKind {
    Ready,
    Running,
    Success,
    Error,
}

enum Screen {
    Menu { selected: usize },
    Action(ActionView),
}

struct App {
    screen: Screen,
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
        WizardAction::GeneratePad => "Create a shared pad",
        WizardAction::QuickEncrypt => "Encrypt file with a new single-use pad",
        WizardAction::QuickDecrypt => "Decrypt file with its single-use pad",
        WizardAction::PadEncrypt => "Encrypt file with a shared pad",
        WizardAction::PadDecrypt => "Decrypt file with a shared pad",
        WizardAction::PadMessageEncrypt => "Encrypt a text message",
        WizardAction::PadMessageDecrypt => "Decrypt a text message",
        WizardAction::PadBalance => "Check remaining pad bytes",
        WizardAction::Quit => "Quit",
    }
}

fn action_description(action: WizardAction) -> &'static str {
    match action {
        WizardAction::GeneratePad => "Start here: create a pad and share a copy securely. Use a different pad for each sending direction. Existing pads will not be replaced.",
        WizardAction::PadBalance => "Check how many bytes are left in the selected pad. Keep its .idx file: deleting or restoring an old index can cause dangerous byte reuse.",
        WizardAction::PadEncrypt => "Choose a file and your outgoing pad. Consumes the file size plus 32 bytes for the legacy hash check. Send only the resulting .glyphs file.",
        WizardAction::PadDecrypt => "Choose a .glyphs file and the sender's matching pad. Restores the file and advances the local pad index. The legacy hash check is not a standard MAC.",
        WizardAction::PadMessageEncrypt => "Uses one pad byte per UTF-8 message byte. Copy the entire result, including its header. This legacy text format has NO authentication.",
        WizardAction::PadMessageDecrypt => "Paste the full encrypted message, including its header. Plaintext is displayed but not automatically saved. This format cannot detect tampering.",
        WizardAction::QuickEncrypt => "Creates a separate pad as long as this file. Share that pad securely, separately from the .glyph ciphertext. This format has NO authentication.",
        WizardAction::QuickDecrypt => "Requires the matching .glyphkey.bin pad; it can be auto-selected beside the ciphertext. Wrong pads or tampering are not detected.",
        WizardAction::Quit => "Close the app. Keep your pads secret and retain their current index files.",
    }
}

fn uses_shared_pad(action: WizardAction) -> bool {
    matches!(action, WizardAction::PadEncrypt | WizardAction::PadDecrypt
        | WizardAction::PadMessageEncrypt | WizardAction::PadMessageDecrypt | WizardAction::PadBalance)
}

fn build_action_view(action: WizardAction) -> io::Result<ActionView> {
    let mut fields = match action {
        WizardAction::GeneratePad => vec![
            InputField::new("Size", "1 MiB".into(), false),
            InputField::new("Save as", DEFAULT_PAD_PATH.into(), false),
        ],
        WizardAction::QuickEncrypt | WizardAction::PadEncrypt => vec![
            InputField::new("File", String::new(), false),
        ],
        WizardAction::QuickDecrypt => vec![
            InputField::new("Encrypted file", String::new(), false),
            InputField::new("Pad key", String::new(), false),
        ],
        WizardAction::PadDecrypt => vec![
            InputField::new("Encrypted file", String::new(), false),
        ],
        WizardAction::PadMessageEncrypt => vec![
            InputField::new("Message", String::new(), true),
        ],
        WizardAction::PadMessageDecrypt => vec![
            InputField::new("Encrypted message", String::new(), true),
        ],
        WizardAction::PadBalance | WizardAction::Quit => Vec::new(),
    };
    if uses_shared_pad(action) {
        fields.push(InputField::new("Shared pad", DEFAULT_PAD_PATH.into(), false));
    }
    if let Some(field) = fields.first_mut() {
        field.select_all = !field.value.is_empty();
    }
    let focus = if fields.is_empty() { Focus::Back } else { Focus::Fields };
    Ok(ActionView {
        action,
        fields,
        selected: 0,
        status: Vec::new(),
        busy: false,
        pad_progress: None,
        available: Vec::new(),
        focus,
        candidate_idx: 0,
        output_panel: None,
        status_kind: StatusKind::Ready,
        preset_idx: 0,
        browse_field: 0,
        picker_dir: env::current_dir()?,
        picker_error: None,
        output_scroll: 0,
    })
}

fn run_action_now(view: &mut ActionView) {
    if view.busy {
        return;
    }
    view.status.clear();
    view.output_panel = None;
    view.output_scroll = 0;
    view.status_kind = StatusKind::Error;

    let pad_path = if uses_shared_pad(view.action) {
        let path = view.fields.last().unwrap().value.trim().to_string();
        if path.is_empty() {
            push_status(view, "Choose the shared pad file first.");
            return;
        }
        path
    } else {
        String::new()
    };
    let pad = pad_path.as_str();

    match view.action {
        WizardAction::GeneratePad => {
            let length_str = view.fields[0].value.trim();
            let target_len = match parse_pad_size(length_str) {
                Ok(v) => v,
                Err(err) => {
                    push_status(view, err.to_string());
                    return;
                }
            };
            let path = view.fields[1].value.trim().to_string();
            if path.is_empty() {
                push_status(view, "Provide a new pad path.");
                return;
            }

            match PadProgress::new(target_len, &path) {
                Ok(progress) => {
                    view.status_kind = StatusKind::Running;
                    view.pad_progress = Some(progress);
                    view.busy = true;
                    push_status(view, format!("Creating {} at {path}", format_bytes(target_len)));
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
                    view.status_kind = StatusKind::Success;
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
                Ok(path) => {
                    view.status_kind = StatusKind::Success;
                    push_status(view, format!("Decrypted to {}", path.display()));
                }
                Err(err) => push_status(view, format!("Decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadEncrypt => {
            let file = view.fields[0].value.trim();
            if file.is_empty() {
                push_status(view, "Provide a file path.");
                return;
            }
            view.busy = true;
            match pad_encrypt(file, pad) {
                Ok(result) => {
                    view.status_kind = StatusKind::Success;
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
            let enc = view.fields[0].value.trim();
            if enc.is_empty() {
                push_status(view, "Provide the encrypted file path.");
                return;
            }
            view.busy = true;
            match pad_decrypt(enc, pad) {
                Ok(result) => {
                    view.status_kind = StatusKind::Success;
                    push_status(view, format!("Decrypted to {}", result.output_path.display()));
                }
                Err(err) => push_status(view, format!("Pad decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadMessageEncrypt => {
            let msg = &view.fields[0].value;
            if msg.trim().is_empty() {
                push_status(view, "Provide a message to encrypt.");
                return;
            }
            view.busy = true;
            match pad_message_encrypt(pad, msg) {
                Ok((cipher, start, end)) => {
                    view.status_kind = StatusKind::Success;
                    push_status(view, format!("Pad bytes used: {}-{}", start, end));
                    push_status(view, "Encrypted message is in the results panel. PgUp/PgDn to scroll.");
                    view.output_panel = Some(("Encrypted message".to_string(), cipher.clone()));
                    append_message_pad("ENCRYPTED", &cipher);
                    push_status(view, format!("Saved to {MESSAGE_PAD_PATH}"));
                }
                Err(err) => push_status(view, format!("Message encryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadMessageDecrypt => {
            let msg = &view.fields[0].value;
            if msg.trim().is_empty() {
                push_status(view, "Provide the glyph message.");
                return;
            }
            view.busy = true;
            match pad_message_decrypt(pad, msg) {
                Ok((plain, start, end)) => {
                    view.status_kind = StatusKind::Success;
                    push_status(view, format!("Pad bytes consumed: {}-{}", start, end));
                    push_status(view, "Decrypted message is in the results panel. PgUp/PgDn to scroll.");
                    view.output_panel = Some(("Decrypted message".to_string(), plain.clone()));
                    push_status(view, "Plaintext has NOT been saved to disk.");
                }
                Err(err) => push_status(view, format!("Message decryption failed: {err}")),
            }
            view.busy = false;
        }
        WizardAction::PadBalance => {
            view.busy = true;
            match pad_balance(pad) {
                Ok((used, total)) => {
                    view.status_kind = StatusKind::Success;
                    let remaining = total.saturating_sub(used);
                    push_status(view, format!("Pad: {pad}"));
                    push_status(view, format!("Total: {}", format_bytes(total)));
                    push_status(view, format!("Used: {}", format_bytes(used)));
                    push_status(view, format!("Remaining: {}", format_bytes(remaining)));
                }
                Err(err) => push_status(view, format!("Unable to read pad balance: {err}")),
            }
            view.busy = false;
        }
        WizardAction::Quit => {}
    }
}

fn run_wizard() -> io::Result<()> {
    let result = run_wizard_inner();
    let _ = crossterm::execute!(io::stdout(), event::DisableMouseCapture, event::DisableBracketedPaste);
    ratatui::restore();
    result
}

fn run_wizard_inner() -> io::Result<()> {
    use ratatui::Terminal;

    let mut terminal: Terminal<_> = ratatui::init();
    crossterm::execute!(io::stdout(), event::EnableMouseCapture, event::EnableBracketedPaste)?;
    let mut app = App {
        screen: Screen::Menu { selected: 0 },
    };

    loop {
        let mut area = ratatui::layout::Rect::default();
        terminal.draw(|f| {
            area = f.area();
            match &app.screen {
                Screen::Menu { selected } => draw_menu(f, *selected),
                Screen::Action(view) => draw_action(f, view),
            }
        })?;

        if let Screen::Action(view) = &mut app.screen {
            let mut fail_message: Option<String> = None;
            let mut complete: Option<(usize, String)> = None;
            let mut clear_progress = false;

            if let Some(progress) = &mut view.pad_progress {
                if progress.error.is_none() && !progress.finished {
                    if let Err(err) = progress.step() {
                        view.status_kind = StatusKind::Error;
                        progress.error = Some(err.to_string());
                        fail_message = Some(err.to_string());
                        view.busy = false;
                        clear_progress = true;
                    }
                }
                if progress.finished {
                    complete = Some((progress.done, progress.path.clone()));
                    clear_progress = true;
                }
            }

            if clear_progress {
                view.pad_progress = None;
            }

            if let Some((done, path)) = complete {
                view.status_kind = StatusKind::Success;
                view.busy = false;
                push_status(view, format!("Created {} at {path}. Share securely; never reuse bytes.", format_bytes(done)));
            }
            if let Some(err) = fail_message {
                push_status(view, format!("Generation failed: {}", err));
            }
        }

        let timeout = if matches!(&app.screen, Screen::Action(view) if view.busy) {
            Duration::from_millis(16)
        } else {
            Duration::from_millis(250)
        };
        if event::poll(timeout)? {
            let mut input = event::read()?;
            if let Screen::Action(view) = &mut app.screen {
                if handle_action_event(view, &input, area) {
                    let selected = WIZARD_ACTIONS.iter().position(|a| *a == view.action).unwrap_or(0);
                    app.screen = Screen::Menu { selected };
                }
                continue;
            }
            if let (Screen::Menu { selected }, Event::Mouse(mouse)) = (&mut app.screen, &input) {
                let list = interface::menu_layout(area)[1];
                let inside = list.inner(ratatui::layout::Margin { horizontal: 1, vertical: 1 });
                if mouse.kind == event::MouseEventKind::Down(event::MouseButton::Left)
                    && inside.contains((mouse.column, mouse.row).into()) {
                    let index = interface::menu_offset(*selected, list) + (mouse.row - inside.y) as usize;
                    if index < WIZARD_ACTIONS.len() {
                        *selected = index;
                        input = Event::Key(event::KeyEvent::new(KeyCode::Enter, KeyModifiers::NONE));
                    }
                } else if mouse.kind == event::MouseEventKind::ScrollDown {
                    *selected = (*selected + 1).min(WIZARD_ACTIONS.len() - 1);
                } else if mouse.kind == event::MouseEventKind::ScrollUp {
                    *selected = selected.saturating_sub(1);
                }
            }
            if let Event::Key(key) = input {
                if key.kind != KeyEventKind::Press {
                    continue;
                }
                match &mut app.screen {
                    Screen::Menu { selected } => match key.code {
                        KeyCode::Char('q') | KeyCode::Esc => {
                            return Ok(());
                        }
                        KeyCode::Up => {
                            if *selected > 0 {
                                *selected -= 1;
                            }
                        }
                        KeyCode::Down => {
                            if *selected + 1 < WIZARD_ACTIONS.len() {
                                *selected += 1;
                            }
                        }
                        KeyCode::Enter => {
                            let chosen = WIZARD_ACTIONS[*selected];
                            if chosen == WizardAction::Quit {
                                return Ok(());
                            }
                            match build_action_view(chosen) {
                                Ok(view) => app.screen = Screen::Action(view),
                                Err(err) => {
                                    return Err(err);
                                }
                            }
                        }
                        _ => {}
                    },
                    Screen::Action(_) => {}
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
        let mut pad_path = DEFAULT_PAD_PATH.to_string();

        while let Some(arg) = args.next() {
            match arg.as_str() {
                "-length" | "--length" => {
                    if let Some(val) = args.next() {
                        match parse_pad_size(&val) {
                            Ok(v) => pad_length = Some(v),
                            Err(err) => {
                                eprintln!("{err}");
                                std::process::exit(1);
                            }
                        }
                    } else {
                        eprintln!("Missing value for -length");
                        std::process::exit(1);
                    }
                }
                "--pad" => {
                    pad_path = args.next().filter(|path| !path.trim().is_empty()).unwrap_or_else(|| {
                        eprintln!("Missing path for --pad");
                        std::process::exit(1);
                    });
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

        match pad_length {
            Some(length) => generate_pad_headless(OsRng, length, &pad_path),
            None => Err(io::Error::new(io::ErrorKind::InvalidInput, "Specify --length when generating a pad outside the wizard")),
        }
    };

    if let Err(err) = result {
        eprintln!("Hieroglyph error: {err}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    pub(super) struct TestDir(PathBuf);

    impl TestDir {
        pub(super) fn new() -> Self {
            let path = env::temp_dir().join(format!("hieroglyph-ux-{}-{}", std::process::id(), OsRng.next_u64()));
            std::fs::create_dir(&path).unwrap();
            Self(path)
        }

        pub(super) fn path(&self, name: &str) -> String {
            self.0.join(name).to_str().unwrap().to_string()
        }
    }

    impl Drop for TestDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn readable_pad_sizes_and_limits() {
        for (input, size) in [("1", 1), (" 32 B ", 32), ("64KiB", 65536), ("10 mib", 10485760), ("1 GiB", 1073741824)] {
            assert_eq!(parse_pad_size(input).unwrap(), size);
        }
        for input in ["", "0", "-1", "1.5 MiB", "10 MB", "1e3", "1 KiB extra", "💡", "184467440737095516160", "18446744073709551615 GiB"] {
            assert!(parse_pad_size(input).is_err(), "{input}");
        }
        assert_eq!(format_bytes(0), "0 bytes");
        assert_eq!(format_bytes(1024), "1.0 KiB (1024 bytes)");
    }

    #[test]
    fn new_pad_never_replaces_existing_pad_or_orphaned_index() {
        let dir = TestDir::new();
        let path = dir.path("outgoing.pad");
        generate_pad_headless(OsRng, 128, &path).unwrap();
        let original = std::fs::read(&path).unwrap();
        assert_eq!(original.len(), 128);
        assert_eq!(PadWriter::new(&path).err().unwrap().kind(), io::ErrorKind::AlreadyExists);
        assert_eq!(std::fs::read(&path).unwrap(), original);
        let other = dir.path("old.pad");
        let idx = read_pad_index_path(Path::new(&other));
        std::fs::write(&idx, b"64").unwrap();
        assert!(PadWriter::new(&other).is_err());
        assert!(!Path::new(&other).exists());
        assert_eq!(std::fs::read(idx).unwrap(), b"64");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            assert_eq!(std::fs::metadata(path).unwrap().permissions().mode() & 0o777, 0o600);
        }
    }

    #[test]
    fn generation_progress_uses_selected_path() {
        let dir = TestDir::new();
        let path = dir.path("incoming.pad");
        let mut progress = PadProgress::new(5000, &path).unwrap();
        while !progress.finished {
            progress.step().unwrap();
        }
        assert_eq!(progress.ratio(), 1.0);
        assert_eq!(progress.path, path);
        assert_eq!(std::fs::metadata(path).unwrap().len(), 5000);
    }

    #[test]
    fn message_budget_counts_utf8_and_preserves_index_on_exhaustion() {
        let dir = TestDir::new();
        let pad = dir.path("message.pad");
        generate_pad_headless(OsRng, 4, &pad).unwrap();
        let (encrypted, start, end) = pad_message_encrypt(&pad, "é").unwrap();
        assert_eq!((start, end), (0, 2));
        assert_eq!(pad_message_decrypt(&pad, &encrypted).unwrap().0, "é");
        let error = pad_message_encrypt(&pad, "abc").unwrap_err().to_string();
        assert!(error.contains("need 3, have 2"));
        assert_eq!(pad_balance(&pad).unwrap(), (2, 4));
        assert!(checked_pad_end(&pad, usize::MAX, 1).is_err());
    }

    #[test]
    fn file_budget_includes_hash_bytes_and_legacy_round_trip_works() {
        let dir = TestDir::new();
        let pad = dir.path("file.pad");
        let input = dir.path("message.txt");
        generate_pad_headless(OsRng, 35, &pad).unwrap();
        std::fs::write(&input, b"abc").unwrap();
        let encrypted = pad_encrypt(&input, &pad).unwrap();
        assert_eq!((encrypted.start, encrypted.end), (0, 35));
        let decrypted = pad_decrypt(encrypted.output_path.to_str().unwrap(), &pad).unwrap();
        assert_eq!(std::fs::read(decrypted.output_path).unwrap(), b"abc");
        let error = pad_encrypt(&input, &pad).err().unwrap().to_string();
        assert!(error.contains("need 35, have 0"));
        assert_eq!(pad_balance(&pad).unwrap(), (35, 35));
    }

    #[test]
    fn shared_pad_actions_offer_an_editable_pad_path() {
        for action in WIZARD_ACTIONS.iter().copied().filter(|a| uses_shared_pad(*a)) {
            let view = build_action_view(action).unwrap();
            assert_eq!(view.fields.last().unwrap().value, DEFAULT_PAD_PATH);
            assert_eq!(view.focus, Focus::Fields);
        }
        let mut field = InputField::new("Path", "old.pad".into(), false);
        interface::handle_char_input(&mut field, KeyCode::Char('u'), KeyModifiers::CONTROL);
        assert!(field.value.is_empty());
    }

    #[test]
    fn balance_action_reads_selected_pad() {
        let dir = TestDir::new();
        let path = dir.path("chosen.pad");
        generate_pad_headless(OsRng, 1024, &path).unwrap();
        let mut view = build_action_view(WizardAction::PadBalance).unwrap();
        view.fields[0].value = path.clone();
        run_action_now(&mut view);
        assert!(view.status.iter().any(|s| s == &format!("Pad: {path}")));
        assert!(view.status.iter().any(|s| s == "Remaining: 1.0 KiB (1024 bytes)"));
    }

    #[test]
    fn decrypted_message_action_uses_selected_pad_and_displays_plaintext() {
        let dir = TestDir::new();
        let path = dir.path("chosen.pad");
        generate_pad_headless(OsRng, 128, &path).unwrap();
        let (encrypted, _, _) = pad_message_encrypt(&path, "private message").unwrap();
        let mut view = build_action_view(WizardAction::PadMessageDecrypt).unwrap();
        view.fields[0].value = encrypted;
        view.fields[1].value = path;
        run_action_now(&mut view);
        assert_eq!(view.output_panel.unwrap().1, "private message");
        assert!(view.status.iter().any(|s| s.contains("NOT been saved")));
        assert!(!view.status.iter().any(|s| s.contains(MESSAGE_PAD_PATH)));
    }

    #[test]
    fn menus_scroll_to_every_action_and_show_help() {
        use ratatui::{backend::TestBackend, Terminal};
        for (width, height) in [(100, 30), (55, 16), (20, 6)] {
            let mut terminal = Terminal::new(TestBackend::new(width, height)).unwrap();
            for (selected, action) in WIZARD_ACTIONS.iter().enumerate() {
                terminal.draw(|frame| draw_menu(frame, selected)).unwrap();
                let view = build_action_view(*action).unwrap();
                terminal.draw(|frame| draw_action(frame, &view)).unwrap();
            }
        }
        let mut terminal = Terminal::new(TestBackend::new(100, 16)).unwrap();
        terminal.draw(|frame| draw_menu(frame, WIZARD_ACTIONS.len() - 1)).unwrap();
        let text: String = terminal.backend().buffer().content.iter().map(|cell| cell.symbol()).collect();
        assert!(text.contains("Quit"));
        assert!(text.contains("How it works"));
        assert!(!text.contains("Glyph drift"));
    }
}
