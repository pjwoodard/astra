# motor-gui — live telemetry visualiser

A tiny host GUI that plots the firmware's 10 Hz `defmt` telemetry (the same
lines `probe-rs run` prints to your terminal) as scrolling charts.

It plots: mechanical rpm, d/q currents (`id`, `iq`, `iq*`), phase currents
(`ia`, `ib`), observer vs open-loop electrical speed (`w_obs`, `w_ol` — watch
them converge at handoff), bus voltage, and worst-case ISR time. The header
shows mode, rpm, VBUS, pot %, θ and a live/stale indicator.

## Run it

```bash
cd tools/motor-gui
cargo run --release          # G474 board (default): spawns `probe-rs run`, plots live
cargo run --release -- --h5  # NUCLEO-H533RE build instead
```

That flashes + streams just like `cargo run` in the firmware dir, so use *one*
of them at a time — the ST-Link only allows a single probe session. `--h5` just
picks the H533 chip + the `thumbv8m…` ELF path; the telemetry format (and hence
everything the GUI plots) is identical across both boards. Build that ELF first
with the firmware's H5 command (see the top-level README).

If you'd rather feed it yourself (or replay a captured log), pipe into it — it
auto-detects a redirected stdin:

```bash
# from the firmware directory, into the built GUI binary:
cargo run --release | tools/motor-gui/target/x86_64-unknown-linux-gnu/release/motor-gui --stdin

# replay a saved capture:
motor-gui --stdin < captured.log
```

## Flags

| Flag | Meaning |
|---|---|
| `--g4` | target the NUCLEO-G474RE build (default: chip `STM32G474RETx`, `thumbv7em…` ELF) |
| `--h5` | target the NUCLEO-H533RE build (chip `STM32H533RETx`, `thumbv8m…` ELF) |
| `--stdin` | read telemetry from stdin instead of spawning probe-rs |
| `--attach` | use `probe-rs attach` (don't reflash) instead of `run` |
| `--print` | headless: parse stdin and print each sample, no window (handy for debugging the parser) |
| `--elf <PATH>` | firmware ELF for probe-rs (overrides the preset) |
| `--chip <NAME>` | target chip (overrides the preset) |

## How it works

`src/parse.rs` matches one telemetry line with a regex (robust to the
timestamp/level prefix; non-telemetry lines are ignored) and returns a
`Sample`. `src/main.rs` reads the stream on a background thread and
`egui_plot` draws a rolling ~90 s window. That's the whole thing.

> This is a **host** binary. The firmware's `.cargo/config.toml` pins the MCU
> target, so this directory has its own `.cargo/config.toml` pinning the host
> target back — always build it from inside `tools/motor-gui/`.
