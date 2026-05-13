# Hieroglyph

A Rust CLI for one-time-pad workflows: pad generation, file/message encryption, and decryption.

## Run

```bash
cargo run --release                   # launch interactive TUI wizard
cargo run --release -- -length 1024   # generate a 1024-byte pad directly
```

Pad output is written to `pad.bin` as raw bytes.

The wizard now runs inside a `ratatui` interface: use ↑/↓ and Enter to choose an action, `e` to edit fields, `Enter` to run, and `Esc` to return to the menu. Pad generation shows a live progress gauge and glyph stream preview.

Wizard options:
- **Generate pad:** Generates random bytes into `pad.bin` with a clean `ratatui` progress view.
- **Encrypt file:** Uses XOR with a one-time pad the exact length of your file. The encrypted data is written to `<name>.glyph` and the pad is stored as `<name>.glyphkey.bin` (binary).
- **Decrypt file:** Provide the encrypted file and its matching `<name>.glyphkey.bin` to recover the original bytes into `<name>.dec`.
- **Pad encrypt:** Use an existing shared pad (`pad.bin`) as a one-time pad. The tool takes the next unused bytes (tracked in a `.idx` file next to the pad), encrypts your file to `<name>.glyphs`, stamps a header noting the byte range used, and appends an encrypted SHA-256 hash for tamper detection (consumes an extra 32 pad bytes per file).
- **Pad decrypt:** Given a pad-encrypted file and the shared pad, it reads the header to grab the right byte range, verifies the encrypted hash, XORs, and outputs `<name>.glyphs.dec`. The `.idx` file is advanced to avoid reusing pad bytes.

This project is designed for local/offline use where you control both pad generation and key handling.