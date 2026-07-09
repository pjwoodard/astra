# Sensorless FOC — X-NUCLEO-IHM08M1 on NUCLEO-G474RE / NUCLEO-H533RE

Rust firmware that spins a **Turnigy Multistar 2216-800Kv** (14-pole / 7 pole-pair,
sensorless outrunner) with field-oriented control on the ST low-voltage BLDC
expansion board. Builds for the G474 (default) or the H533 — see [Targets](#targets).

```
TIM1 (25 kHz center-aligned, complementary + dead time)
  └─ OC4REF @ peak ──► ADC1/ADC2 injected (phase shunts, low sides conducting)
                              └─ ADC1_2 IRQ = the whole FOC loop:
                                 Clarke ► Park ► PI(id), PI(iq) ► circle limit
                                 ► inv-Park ► SVPWM ► TIM1 CCRs
                                 + Ortega flux observer + PLL (rotor angle/speed)
                                 + 1 kHz slow tick (pot, VBUS, speed PI)
Startup: align (d-axis current) ► open-loop ramp ► observer handoff ► closed loop
```

## ⚠️ Safety first

- **Remove the propeller.** Always. Startup tuning *will* produce sudden torque.
- First runs: **bench supply at ~12 V with a 3–4 A current limit**, not a LiPo.
- Clamp the motor down. Keep fingers, wires and hair away from the bell.
- The firmware trips at 20 A (software) and the board at 30 A (hardware
  comparator), but neither protects *you*.
- Per your own test data: don't run a 12" prop, and avoid 11" SF on 4S — this
  motor overheats there. The green-highlighted combos in your sheet are the
  safe operating points once you do mount a prop.

## Hardware setup

1. Stack the IHM08M1 on the NUCLEO-G474RE via the ST morpho connectors.
2. **3-shunt jumpers:** set J5 and J6 to the 3-shunt position (this firmware
   samples two shunt amplifiers and reconstructs the third phase). See UM1996.
3. Motor's three wires to the motor terminal block (any order; swap any two
   wires to reverse direction).
4. DC supply (10–48 V, start at 12 V) to the power terminal. The expansion
   board can power the Nucleo — check the J9/JVIN jumper per UM1996, and if
   you do that, **remove the Nucleo's own USB-power jumper conflicts** (keep
   ST-Link USB for flashing only).
5. Controls: blue button (B1) = start/stop and fault-clear, on-board pot =
   speed, expansion-board LED = status (off idle, solid running, blinking fault).

### Pin map used (standard IHM08M1 ⇄ Nucleo-64 morpho mapping)

| Signal | MCU pin | Peripheral |
|---|---|---|
| Phase U/V/W high-side (HIN) | PA8 / PA9 / PA10 | TIM1 CH1/2/3, AF6 |
| Phase U/V/W low-side (LIN) | PA7 / PB0 / PB1 | TIM1 CH1N/2N/3N, AF6 |
| Current feedback U / V | PA0 / PC1 | ADC1 IN1 / ADC2 IN7 (injected) |
| Current feedback W | PC0 | (reconstructed as −ia−ib) |
| Bus voltage sense | PA1 | ADC1 IN2 |
| Speed potentiometer | PA4 | ADC2 IN17 |
| User LED / button | PB2 / PC13 | GPIO |

**Verify PA0/PC1/PC0 and PA1 against UM1996 / your board silkscreen before
powering up** — this is the standard mapping ST uses for this board, but it's
the one thing worth double-checking with a multimeter in analog-watch mode.

## Bring-up

See **BRINGUP.md** — staged verification via cargo features (stage0-pwm,
stage1-adc, trip-test, stage3-align, stage3-openloop), with defmt telemetry
and an ISR timing pin on PC10 baked into every build.

## Live GUI (optional)

`tools/motor-gui/` is a small host app that plots the same 10 Hz telemetry as
scrolling charts (rpm, d/q + phase currents, observer vs open-loop speed, VBUS,
ISR time) instead of scrolling text:

```bash
cd tools/motor-gui
cargo run --release          # G474 (default): spawns `probe-rs run` and plots live
cargo run --release -- --h5  # NUCLEO-H533RE build instead
```

Use it *instead of* a bare `probe-rs run` (single ST-Link session). The GUI is
board-agnostic — `--h5` just selects the H533 chip + ELF. See
`tools/motor-gui/README.md` for piping/replay options.

## Targets

The firmware runs on two boards, selected by a cargo feature. All the control
logic (`main.rs`, `foc.rs`, `observer.rs`) is MCU-independent; every peripheral
register access lives in `src/mcu/<device>.rs` behind a common API:

| Feature | Board | Core | SYSCLK | PAC |
|---|---|---|---|---|
| `mcu-g4` *(default)* | NUCLEO-G474RE | Cortex-M4F | 170 MHz | `stm32g4` |
| `mcu-h5` | NUCLEO-H533RE | Cortex-M33 | 250 MHz | `stm32h5` |

Same X-NUCLEO-IHM08M1 shield, same physical pins (the Nucleo-64 header pinout is
identical across the two boards); only the silicon layer differs (clock tree,
ADC generation, TIM1 alternate function AF6→AF1, ADC channel numbers).

## Building and flashing

```bash
cargo install probe-rs-tools          # provides `probe-rs run`

# G474 (default)
rustup target add thumbv7em-none-eabihf
cargo run --release                   # builds + flashes via the on-board ST-Link

# H533
rustup target add thumbv8m.main-none-eabihf
cargo run --release --no-default-features --features mcu-h5 \
    --target thumbv8m.main-none-eabihf
```

Add a bring-up stage with `--features "mcu-h5,stage0-pwm"` etc. `cargo build`
+ your own flasher (STM32CubeProgrammer with the generated ELF) also works.

> **G4** was written against `stm32g4` PAC v0.15.1 (register logic per RM0440).
> **H5** (`stm32h5` v0.16, RM0481) compiles and links to a valid H533 image but
> has **not been run on hardware**. Before driving gates on the H533, confirm
> the two datasheet-derived constant groups flagged at the top of
> `src/mcu/h5.rs` (`AF_TIM1`, the `CH_*` ADC channels) and scope stage 0 first.

## First spin-up procedure

1. Flash, power the board, pot fully counter-clockwise.
2. Press the blue button. You should hear/feel: a firm *align* twitch (~0.7 s),
   then a smooth acceleration to ~700 rpm, then handoff (LED stays solid).
3. If it stutters, reverses briefly, or faults at handoff, flip `CURR_SIGN`
   to `-1.0` in `main.rs` — the current-feedback polarity is the most common
   first-run gotcha.
4. Turn the pot up slowly. Button again to stop.

## Tuning guide (constants at the top of `src/main.rs`)

| Constant | What it does | Symptom if wrong |
|---|---|---|
| `R_PHASE`, `L_PHASE` | Motor model for observer + current-loop gains. **Measure them** (line-to-line ÷ 2). | Observer never locks; handoff fault |
| `FLUX_LINKAGE` | Computed from Kv — usually fine as-is | Speed estimate scaled wrong |
| `I_ALIGN`, `I_STARTUP` | Startup torque | Rotor doesn't follow the ramp → raise; motor cogs hard → lower |
| `OL_ACCEL`, `OMEGA_HANDOFF` | Open-loop ramp | Loses sync before handoff → lower accel or raise handoff speed |
| `OBS_GAMMA` | Observer convergence | Too low: drifts/never locks. Too high: jittery angle at speed |
| `PLL_KP/KI` | Angle-tracking bandwidth | Jitter (too high) vs lag under load (too low) |
| `SPEED_KP/KI` | Speed loop | Oscillating rpm vs sluggish response |
| `I_MAX`, `I_TRIP` | Current limits | Keep `I_MAX` ≤ 10 A until things are dialed in |
| `CURR_AMP_PER_LSB` | Shunt scaling (0.01 Ω × gain 1.53) | Confirm the amplifier gain in UM1996 |

Measuring R and L cheaply: R — push 1 A DC through two motor wires with your
bench supply, read the voltage, R_ll = V/I, `R_PHASE = R_ll / 2`. L — an LCR
meter at 1 kHz across two wires, `L_PHASE = L_ll / 2`.

## Known simplifications (things a production ESC adds)

- No dead-time compensation, no field weakening, no temperature derating.
- The hardware overcurrent comparator (CPOUT → PA12/TIM1_ETR, BKIN on PA6) is
  **not** wired into TIM1's break input yet — software trip only. Wiring
  BKIN/ETR to BDTR's break function is the single best robustness upgrade.
- Two-shunt sampling at the PWM peak (all low sides conducting): current
  reconstruction degrades at duty > ~95 % (already clamped in `svpwm`).
- Open-loop startup is unloaded-rotor tuned; with a prop mounted you'll likely
  need more `I_STARTUP` and gentler `OL_ACCEL`.
