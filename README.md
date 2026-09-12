# Hieroglyph

A local Rust app for finite, shared-pad file and message workflows. The interface
uses plain-language actions; the project name and existing encrypted formats are
unchanged. There is no algorithmic pad expansion or reusable-key encryption mode.

## Run

```bash
cargo run --release                    # open the interactive workspace
cargo run --release -- --help
cargo run --release -- --length "10 MiB" --pad alice-to-bob.pad
```

Sizes accept positive integers in bytes, `KiB`, `MiB`, or `GiB` (case-insensitive).
For example, `1024`, `64KiB`, and `10 MiB` are valid. Quote sizes containing spaces
on the command line. The old `-length` flag still works. Headless generation does
not open a terminal UI; errors return a nonzero exit status.

Pads default to `pad.bin`, but every shared-pad action lets you choose a path.
Generation refuses to overwrite an existing pad or reuse a path with an existing
`.idx` file. On Unix, new pads have owner-only read/write permissions. Secret pad
bytes are never previewed on screen. Keep adequate free disk space; a failed or
interrupted generation can leave a partial file. Use a new path for a retry.

## First shared-pad exchange

1. Choose **Create a shared pad**, enter its size and a fresh path, and run it.
2. Share an identical copy with the recipient through a secure channel. Never
   send the pad alongside ciphertext over an untrusted channel.
3. Use one pad **only for Alice → Bob**. Create and share a completely independent
   pad for **Bob → Alice**. Select the appropriate pad in each action. Separate
   local index files cannot coordinate two simultaneous senders using one pad.
4. Choose **Encrypt file with a shared pad** and select the outgoing pad. Send
   the resulting `.glyphs` file, not the pad or its index.
5. The recipient chooses **Decrypt file with a shared pad**, the received file,
   and their copy of the matching pad.
6. Use **Check remaining pad bytes** to see total, used, and remaining capacity.
   When exhausted, generate and securely share a new pad. Never reset the index.

Keep each pad's current `.idx` file with it. Moving or renaming a used pad without
its index, restoring an old index, or copying a used pad as a "new" pad can cause
byte reuse. Pad/index lifecycle and crash recovery are not fully hardened; this
is not an audited secure-messaging system.

## Interface

The workspace opens with a pixel-style **HIEROGLYPH** wordmark (a compact text
version on smaller terminals). Click a menu item, or use **↑/↓** and **Enter**.

Action screens are forms, not a separate editing mode. For example, **Create a
shared pad** has a **Size:** field, **1 MiB / 10 MiB / 100 MiB** preset buttons, a
**Save as:** field, and clearly labelled **Create pad** and **Back** buttons.
Type a custom size if none of the presets fit. The initial size is selected so
typing immediately replaces it.

- **Click** a field to position its cursor; type normally. No `e` shortcut needed.
- **Tab / Shift+Tab:** move through fields, presets, Browse, action buttons, and results.
- **Enter** in a single-line field moves to the next field, then the primary
  button. It never submits while you are still in a field. In messages, it
  inserts a newline. **Enter / Space** activates a focused button.
- **Ctrl+A:** select the entire field. **Ctrl+U:** clear it. **←/→**, **Home/End**,
  **Backspace**, and **Delete** edit at the cursor; **↑/↓** move within messages.
- **Browse** (or **F2**) opens the file browser for the selected path field.
  Click a file or use **↑/↓ + Enter**. Open folders the same way; **Backspace**
  goes to the parent folder. **Esc / Cancel** closes the browser without selecting.
- Paste text normally; terminals with bracketed paste support keep multiline
  messages intact. Pasting never activates a button.
- **F5:** run the action directly. **Esc / Back:** return to the workspace.
- Results show **Done** or **Needs attention**, not a mixture of old status
  messages. **PgUp/PgDn** or the mouse wheel over results scroll long output.
  Focus the results with Tab or a click to use **↑/↓** as well. To select text
  with the terminal's own mouse selection, many terminals require holding Shift.
- **q:** quit from the workspace. While a pad is being generated, inputs and
  buttons are disabled until it finishes; progress is shown without secret bytes.

Forms use a side-by-side layout on wide terminals and stack results below the
fields on narrower ones. They require at least **60 columns × 24 rows**; smaller
windows show a resize prompt and cannot accidentally activate invisible controls.
Mouse and bracketed-paste modes are disabled on normal exit and returned errors.

Every action explains its inputs, output, and important limitations.
Existing output filenames and encodings remain compatible:

| Workflow | Output | Pad consumption / checks |
| --- | --- | --- |
| File + shared pad | `<stem>.glyphs` | File bytes + 32 bytes; legacy encrypted SHA-256 check |
| Shared-pad file decryption | `<stem>.glyphs.dec` | Uses the recorded byte range; advances local index |
| Text + shared pad | Header and glyph text | One pad byte per UTF-8 byte; **no authentication** |
| File + new single-use pad | `<stem>.glyph` and `<stem>.glyphkey.bin` | One pad byte per file byte; **no authentication** |
| Single-use file decryption | `<stem>.dec` | Requires the matching pad; **no authentication** |

Encrypted text messages are appended to `message_pad.txt`. Decrypted messages
are displayed only, not automatically logged. Old plaintext entries already in
that file are not deleted. Legacy file actions can overwrite their derived output
paths; preserve previous outputs before rerunning them.

## What a one-time pad does—and does not—promise

A true OTP gives *perfect secrecy*: ciphertext alone does not distinguish between
possible equal-length plaintexts, even with unlimited computation. It requires
uniform, independent secret randomness at least as long as the message, used
exactly once. It does not hide message length, protect a compromised endpoint,
or authenticate the sender.

Deterministic expansion cannot create additional secret entropy. To extend a pad
while keeping the OTP guarantee, obtain and securely share fresh independent
randomness. This app uses `OsRng`, the operating system's cryptographic random
generator; it does not certify a source of truly independent random bits or claim
mathematically perfect security for the whole implementation. The legacy
encrypted-hash check is not a standard MAC or a proof of unconditionally secure
authentication; the text and single-use file formats have no integrity check.

Inventing a character or private font does not add secrecy. Computers represent
it as bytes, a code point, or an image. An attacker can label an unfamiliar symbol
"A" and analyze its repetitions without knowing its name or meaning. A secret
symbol mapping is another key, not a way around the entropy requirement.

## Tests

```bash
cargo test
```

Tests cover readable sizes, non-destructive pad creation, custom paths, exhausted
pad errors, legacy round trips, responsive layouts, keyboard and mouse controls,
Unicode editing, paste, file browsing, progress locking, and result scrolling.