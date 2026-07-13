# Air Data System — Design Log

Fast fixed-wing UAV air-data system. This log captures **decisions and rationale**.
Let git track *what* changed in the KiCad files; this log tracks *why*.

**How to use this file**
- Add a decision the moment you make one. Keep it to: Decision / Why / Alternatives rejected / Traces to.
- Never delete a superseded decision — mark it `SUPERSEDED by DEC-xx` and leave it.
- When an open question closes, promote it to a DEC entry.
- Bump the Revision history when a batch of decisions lands in the schematic.
- Commit the KiCad project after each change with a message referencing the DEC/REQ id.

---

## Requirements

| ID | Requirement | Status |
|----|-------------|--------|
| REQ-01 | Differential pressure range ≥ ±53 kPa (max Qc 53 kPa) | Range met; airspeed by subtraction of two ILPS28QSW, MCU-synchronized (DEC-15/25) |
| REQ-02 | Absolute pressure range ≥ 0–103.421 kPa (ceiling 10 km) | **Met:** MS5611 restores altitude (DEC-16); ~±8 m SL, floor problem resolved |
| REQ-03 | Powered by 5 V or 3.3 V | Met (DEC-09) |
| REQ-04 | ~~SPI or I²C~~ → **CAN-FD + 100BASE-TX**, locking connectors | **AMENDED (DEC-25/26/28):** dual interface — FDCAN (H563) + Ethernet TX, each on a powered locking connector |
| REQ-05 | Measure pitot-tube temperature | On-board MAX31865 on host SPI + remote PT1000 (DEC-18/21) |
| REQ-06 | Measure outside air temperature (OAT) | On-board MAX31865 on host SPI + remote probe; SAT via DEC-02 (DEC-18/21) |
| REQ-07 | Pitot heating, 12 V @ 3 A (36 W) | Power board = dumb 12 V/3 A rail; **air-data board MCU switches & controls** pitot heat (DEC-29); feedback = on-board pitot RTD; no-airflow interlock = airspeed |
| REQ-08 | (Soft) Keep pressure sensors > −40 °C at 10.5 km | On-board heated isolated pocket; **MCU-run heater PID** + HW over-temp backstop (DEC-19/20/25); accuracy-critical |
| REQ-09 | Airspeed accuracy near stall (Vs = 50 m/s → Qc ≈ 1.5 kPa) | **±1 m/s NOT guaranteed**; expect ~±1.6 m/s (nominal), accepted for manufacturability (DEC-15) |
| REQ-10 | Static pressure from external ports (barbs), not fuselage interior | **Met:** manifold integrated into machined case (DEC-20) |

---

## Decision log

### DEC-01 — Flight envelope sanity check
- **Decision:** Top speed ~Mach 0.8; 103.421 kPa = 15.000 psia. **Traces to:** REQ-01, REQ-02.

### DEC-02 — OAT corrected from recovery temperature
- **Decision:** Measure recovery temp; compute SAT from Mach in firmware.
- **Why:** At M0.8 probe reads ~25–30 °C above SAT. **Traces to:** REQ-06.

### DEC-05 — Ground auto-zero + isothermal dies for low-Qc accuracy
- **Decision:** Zero the differential at V=0 before flight; hold both dies at the same temperature.
- **Why:** After zero, the airspeed error is dominated by die-to-die gain mismatch × static pressure; isothermal operation removes the differential thermal-drift term and zeroing removes offset mismatch. Both are free and essential to the accuracy in DEC-15.
- **Traces to:** REQ-01/09, REQ-08.

### DEC-06 — External static via barbs, shared reference
- **Decision:** Static line feeds the static sensor and the total sensor's reference; total high side → pitot.
- **Notes:** Moisture trap/drain, slope, ports down, leak-check. **Traces to:** REQ-10, REQ-02.

### DEC-09 — Power: 5 V (ARK PAB peripheral) → 3.3 V LDO
- **Decision:** AP2112K-3.3, EN→5 V. **Traces to:** REQ-03.

### DEC-10 — Interface = I²C
- **Decision:** Bus is I²C (ILPS28QSW is I²C/I3C only). **Traces to:** REQ-04.

### DEC-11 — I²C bus is 3.3 V end-to-end
- **Decision:** No level translation; upstream pull-ups to +3V3. ARK PAB peripheral I²C confirmed 3.3 V.
- **Traces to:** REQ-04, DEC-09.

### DEC-12 — Stall airspeed accuracy target = ±1 m/s
- **Decision:** Target ±1 m/s at stall (Qc budget ≈ 61 Pa). *Now aspirational — see DEC-15 for what's actually achievable with un-binned parts.*
- **Traces to:** REQ-09.

### DEC-15 — Revert to dual ILPS28QSW (subtraction); accept lower airspeed accuracy
- **Decision:** Airspeed = Qc from two ILPS28QSW (static + total) by subtraction; static/altitude = the static ILPS28QSW. Only mitigations: isothermal dies + ground cross-zero (no binning, no chamber cal).
- **Why:** Honeywell differential parts don't have sufficient availability; MS5525DSO has JLCPCB friction. Two ILPS28QSW is the simplest to source and build. Airspeed accuracy is knowingly traded for manufacturability.
- **Expected accuracy (sea level, cross-zero + isothermal, ~0.1 % gain mismatch nominal):**
  - Airspeed: ~±1.6 m/s at 50 m/s (stall) → ~±0.3 m/s near Vmax. Error ~constant in pressure (~100 Pa), so worst at low speed, better at high speed and at altitude. Scales with actual gain mismatch (×0.5 at 0.05 %, ×2 at 0.2 %).
  - Static/altitude: ~±0.5–1 hPa in-band → ±5–8 m (SL), ±14 m (5 km), degraded/unspecified at ceiling (below ~260 hPa floor). Relative altitude sub-meter (noise ~0.3 Pa).
- **Consequences:**
  - **0x5C address collision returns** → 2-ch I²C switch (PCA9540B) back (per DEC-08). Switch adds minor read-skew (~few Pa, negligible vs gain term). LTC4316 translator optional if skew ever matters. *(Later resolved differently — see DEC-17.)*
  - Ceiling static accuracy degraded (ILPS floor) — optional fix: MS5611 for static only (adds a 2nd part type). *(Adopted — see DEC-16.)*
  - Run Mode 1 at low speed; both to Mode 2 above ~180 m/s (total sensor clips 1260 hPa).
  - Ground cross-zero every flight is mandatory (without it, offset mismatch ≈ ±3–5 m/s at stall).
- **Supersedes:** DEC-13, DEC-14. **Re-activates:** DEC-07/DEC-08 topology.
- **Traces to:** REQ-01, REQ-02, REQ-09, REQ-04, DEC-05, DEC-06.

### DEC-16 — Re-add MS5611 for static/altitude
- **Decision:** Sensor set = 2× ILPS28QSW (Qc by subtraction) + 1× MS5611 (static/altitude). All three tap the shared pitot/static manifold.
- **Why:** MS5611's ~10 mbar floor restores ceiling altitude accuracy (~±8 m SL vs ILPS ~±75–100 m at ceiling). Keep both ILPS for Qc — the matched pair is what makes the cross-zero/isothermal accuracy work; do NOT fold MS5611 into the subtraction (mixed-type mismatch).
- **Cost:** +1 part type, +1 manifold port.
- **Supersedes:** DEC-15 static-source (altitude now MS5611, not the static ILPS; static ILPS remains only as the Qc reference).
- **Traces to:** REQ-02, REQ-01.

### DEC-17 — Local MCU → self-contained data unit
- **Decision:** On-board MCU (STM32G0/L4-class) reads all sensors, runs Qc subtraction, cross-zero, TAT→SAT, altitude, and the heater loop; presents finished air-data to the host over I²C (slave).
- **Why:** Makes the unit "just produce data." Its two I²C peripherals put one ILPS on each internal bus → **0x5C collision solved without a switch**; MS5611 (0x76) shares a bus; RTD front-ends on SPI.
- **Consequences:** Host sees the MCU, not raw sensors. Firmware owns the cross-zero trigger, synchronized ILPS sampling, and config/zero storage.
- **Supersedes:** I²C switch (DEC-08) again.
- **Status:** *SUPERSEDED by DEC-21* — no on-board MCU; host does the math.
- **Traces to:** REQ-04, REQ-01, DEC-05, DEC-02.

### DEC-18 — On-board temperature front-ends (pitot RTD + OAT)
- **Decision:** Two MAX31865 RTD front-ends (SPI) on this board, with connectors for remote PT1000 probes — pitot skin temp, and OAT in the airstream (shielded). SAT from OAT + Mach (DEC-02).
- **Note:** Probes are necessarily external; the unit houses the front-ends.
- **Supersedes:** the "separate board" framing for REQ-05/06. **Traces to:** REQ-05, REQ-06, DEC-02.

### DEC-19 — On-board sensor-pocket heater (accuracy-critical)
- **Decision:** Heater in the sensor pocket — 12 V in (a few watts) via an on-board low-side MOSFET. Control method settled in DEC-23 (flight-computer switched). Adds a 12 V input connector to the board.
- **Why:** Holding the dies at a constant setpoint is what keeps the ground cross-zero valid → the ±0.3 vs ±1.6 m/s hinge (DEC-05/12). Now hard, not soft.
- **Open:** whether the pitot heater's 12 V/3 A also routes through this unit (size connector/traces) or stays on the power board.
- **Traces to:** REQ-08, DEC-05, REQ-07.

### DEC-20 — Mechanical: machined case = manifold + isolated heated pocket
- **Decision:** Machined case integrates the pitot/static pneumatics (barbed case ports → internal channels → O-ring seals to each of the 3 sensor ports). The sensor pocket is **thermally isolated** from the aluminum (insulated island) so the heater can hold setpoint against the case's heatsinking.
- **Why:** Aluminum is a great heatsink and would fight the heater; isolation is required for the accuracy story. Conformal-coat the board; drain condensate; gasket the case.
- **Traces to:** REQ-10, REQ-08, DEC-06, DEC-19.

### DEC-21 — No on-board MCU; flight computer does the air-data math
- **Decision:** Board presents raw sensors to the host; the flight computer runs Qc subtraction, cross-zero, TAT→SAT, and altitude. No MCU on the board.
- **Why:** Flight computer has ample cycles; simpler board, no firmware to maintain.
- **Consequences:**
  - **0x5C collision returns** → separate the two ILPS on the host I²C: **dual host I²C bus** (preferred — lowest Qc-sample skew, no extra part) or LTC4316 translator, vs PCA9540B switch (simplest, but host channel-switching adds read-skew). Choose per PAB I²C availability.
  - RTD front-ends (MAX31865) are **SPI** → board exposes I²C (pressure) + SPI (RTD) to the host; or swap to an I²C ADC + RTD bridge to stay single-bus.
  - **Heater loop stays on-board as an autonomous thermostat** (comparator/thermostat IC + MOSFET), independent of the host, so the pocket holds setpoint through host reboots/latency. Host may optionally enable/monitor via GPIO.
  - Host owns synchronized ILPS one-shot triggering (skew-sensitive → favors dual-bus/translator) and cross-zero storage.
- **Supersedes:** DEC-17 (local MCU). **Re-opens:** address separation (DEC-08).
- **Traces to:** REQ-04, REQ-01, REQ-08, DEC-05, DEC-15.

### DEC-22 — Host buses confirmed: 1× I²C + 1× SPI; LTC4316 for 0x5C separation
- **Decision:** PAB peripheral exposes one I²C and one SPI. Pressure sensors on I²C: both ILPS28QSW share it via an **LTC4316 address translator** offsetting one unit; MS5611 (0x76) on the same bus. Both MAX31865 on SPI.
- **Why:** Single I²C → the two 0x5C ILPS still collide; LTC4316 keeps both live at different addresses for **back-to-back reads** (low Qc-sample skew), unlike a switch that serializes with a host channel-select per read.
- **Alternatives rejected:** PCA9540B switch (host channel-switching adds read-skew — bad for synchronized Qc); dual host I²C bus (not available — only one I²C).
- **Closes:** address-separation + PAB-bus open questions.
- **Traces to:** REQ-04, REQ-01, DEC-21.

### DEC-23 — Pocket heater switched by the flight computer (dumb 12 V power board)
- **Decision:** Power board is a dumb 12 V rail (no MCU). The flight computer runs the pocket-heater loop and drives an **on-board low-side MOSFET** via one logic (PWM/enable) line. **Feedback = the ILPS/MS5611 die temperatures over the existing I²C** (regulate the two ILPS dies to setpoint) — no dedicated loop sensor.
- **Required safety (independent of host):**
  - Gate **pull-down → default OFF** when the control line is undriven (host booting/hung).
  - **Hardware over-temp cutoff** (thermal switch or comparator on a cheap thermistor) kills the heater ~+80 °C regardless of the host (MS5611 caps +85 °C). Only dumb thermal sensor left on the board.
- **Host firmware:** gate the cross-zero on "dies at setpoint & stable"; on a host reboot/hang the pocket cools → treat the stored zero as stale, re-zero or flag airspeed degraded.
- **Cable:** 12 V+GND (power board) + one control line (flight computer, with I²C/SPI). MOSFET local → no PWM'd 12 V on the harness. Single-point tie between 12 V return and logic ground.
- **Setpoint:** ~+45 °C, firmware-tunable.
- **Residual risk:** temperature hold now depends on host availability; covered by the two safety items + firmware zero-gate.
- **Supersedes:** the autonomous-thermostat control from DEC-19/21.
- **Traces to:** REQ-08, DEC-05, DEC-12, DEC-21.

### DEC-24 — RTD type locked: PT1000, 3-wire, MAX31865 + 4300 Ω 0.1 % reference
- **Decision:** Both channels PT1000, 3-wire, each on a MAX31865 with a 4300 Ω (4× RTD) 0.1 %, ≤25 ppm/°C reference resistor.
- **Why:** PT1000 → lead resistance & self-heating less significant (remote probes, thin air); 3-wire adequate since OAT system error is dominated by TAT→SAT/recovery, not the readout; the reference resistor is the accuracy-critical part.
- **Closes:** RTD type/wiring open item.
- **Traces to:** REQ-05, REQ-06, DEC-18.

### DEC-25 — Add STM32G0B1 MCU with CAN; back to a smart node
- **Decision:** On-board STM32 (Cortex-M0+, LQFP48, FDCAN) reads all sensors, runs Qc / cross-zero / TAT→SAT / altitude and the pocket-heater PID, and talks CAN to the flight computer. Reverses the no-MCU choice (DEC-21). **Part: STM32C092CB** (ST's low-cost C0 line, 2× I²C / 2× SPI / FDCAN / SWD, −40…+125 °C) — undercuts the G0B1 while keeping the ST toolchain + CAN-FD. **G0B1CBT6 (~$1.39, confirmed in stock) is the fallback** if LCSC hasn't stocked the C092CB for assembly.
- **Why:** MCU makes the heater loop autonomous (kills the DEC-23 host-availability risk) and its two I²C peripherals separate the two 0x5C ILPS → **LTC4316 removed**. CAN is robust over the harness and native to the DroneCAN/ARK ecosystem. (Only G0B1/G0C1 in the G0 line have FDCAN.)
- **Peripheral map:** I2C1 = ILPS#1 + MS5611; I2C2 = ILPS#2; SPI1 = 2× MAX31865; FDCAN1 = transceiver; TIM-PWM = heater; ADC/die-temp = heater feedback; SWD.
- **New parts:** MCU (STM32C092CB / G0B1 fallback), CAN transceiver (SN65HVD230 classic/DroneCAN, or FD part for CAN-FD — C092 FDCAN supports both), external crystal (CAN bit timing, cold), SWD header, CAN connector (DroneCAN 4-wire = data + 5 V).
- **Removed:** LTC4316 + translated-bus pull-ups; host I²C+SPI 10-pin connector; host heater-control line.
- **Requirement impact:** amends REQ-04 (SPI/I²C → CAN).
- **Supersedes:** DEC-21 (no-MCU), DEC-22 (LTC4316 / host buses); **amends** DEC-23 (heater control → MCU PID; HW backstop retained).
- **CAN (resolved):** CAN-FD (fastest); single DroneCAN 4-pin connector, **bus-powered**; on-board stuff-option 120 Ω termination (single connector ⇒ end node). Transceiver **SIT1044T/3** (~$0.23, FD, 3.3 V IO); TCAN1044V alt for 8 Mbps + ±58 V fault/12 kV ESD. *FD rate is realized only if the flight-computer/bus side also runs FD — else classic fallback.*
- **Traces to:** REQ-04, REQ-01, REQ-08, DEC-05, DEC-02.

### DEC-26 — Two powered connectors (FDCAN + 100BASE-TX); C593 committed
- **Decision:** Two locking connectors — **FDCAN** (DroneCAN 4-pin: CANH/CANL/+5 V/GND) and **100BASE-TX** (6-pin: TX±/RX±/+5 V/GND). Both feed +5 V into a **TPS2116 ideal-diode power mux** (~$0.22 LCSC): power redundancy (runs on either), reverse/backfeed blocking, ~55 mΩ (~15 mV drop vs the Schottky's 0.35 V), with priority/automatic switchover set by the PR pin. TVS on the rail. *(Chose the mux over 2× LM66100 ideal diodes for clean priority switchover between two equal 5 V sources.)*
- **Ethernet subsystem:** RMII PHY **LAN8742A** + 25 MHz crystal + magnetics (discrete transformer default; transformerless optional since same airframe) + the 6-pin connector.
- **MCU:** Ethernet MAC + FDCAN required → **STM32C593 committed** (C092/G0B1 have no Ethernet). Eth+CAN fallback if C593 is unstocked: **STM32F407** (mature, in stock, has both).
- **Findings driving it:** ARK PAB Ethernet is standard **100BASE-TX** on a 4-pin JST-GH (ARK sells a passive JST-GH→RJ45 adapter) → **100BASE-T1 ruled out** (would need a media converter). The 6-pin Eth connector adds power, so it is **non-ARK-standard → custom cable**.
- **Supersedes:** DEC-25 part choice (C092 → C593); Rev D single-connector CAN-only.
- **Traces to:** REQ-04, REQ-03, REQ-01.

### DEC-27 — MCU = STM32F407 (mature/in-stock); CAN reverts to classic
- **Decision:** MCU = **STM32F407VET6** (LQFP100, ~$2.83, in stock at LCSC). Has the RMII Ethernet MAC + classic **bxCAN** (2× CAN 2.0B). Resolves the C593 stock risk by using the mature, stocked Ethernet+CAN STM32.
- **Consequence — CAN reverts to classic CAN 2.0B (no CAN-FD):** the F4 predates FDCAN. Acceptable because the ARK/DroneCAN bus is classic anyway; confirm no hard FD requirement. The SIT1044 FD transceiver still works (superset), or simplify to a classic SN65HVD230.
- **Also:** LQFP100 (ample pins — no budget concern; board grows slightly); most mature STM32 (best DroneCAN/example support).
- **Supersedes:** DEC-26 C593 commitment; **reverts the CAN-FD choice** in DEC-25/26 to classic CAN.
- **Traces to:** REQ-04, REQ-01.
- **Schematic:** Rev G pending (swap C593→F407, LQFP100 pinout, classic CAN) — awaiting "go".

### DEC-28 — MCU = STM32H563 (in-stock FDCAN + Ethernet); CAN-FD restored
- **Decision:** MCU = **STM32H563RIT6** (Arm Cortex-M33 @ 250 MHz, 1 MB flash, LQFP64, ~$4.10 LCSC, in stock in the 100s) — RMII Ethernet MAC + **2× FDCAN**. Use VIT6 (LQFP100, ~$4.71) if the pin budget is tight.
- **Why:** the FDCAN+Ethernet part you can actually buy — what the C593 aimed to be but scarce. Near F107 price, far stronger core, in stock for JLCPCB.
- **Effect:** **restores CAN-FD** (undoes DEC-27's classic-CAN revert); the SIT1044 FD transceiver fits again. Rest of the board unchanged (dual ILPS, MS5611, 2× MAX31865, LAN8742 TX, LM66100 diode-OR, heater, LDO).
- **Note:** newer than the F4 but shipping since 2023, full CubeMX support — mature enough for a flight node.
- **Supersedes:** DEC-27 (F407). **Traces to:** REQ-04, REQ-01.
- **Schematic:** Rev G pending (H563 + CAN-FD + LAN8742 + SIT1044 + LM66100 OR) — awaiting "go".

### DEC-29 — Air-data board owns the pitot heater (switcher + control + interlock)
- **Decision:** The on-board MCU (H563) switches and controls the pitot heater — not the flight computer. Adds a **3 A-rated low-side MOSFET** (MCU PWM) + **independent hardware over-temp cutoff**. Feedback = the on-board pitot RTD; **no-airflow protection** = airspeed from the pressure sensors (this board uniquely has both signals). Power board reverts to a dumb 12 V/3 A rail.
- **Why:** closes the unowned pitot-overheat gap (open Q2) with a smarter backstop than any off-board option — it can cut/limit power on genuine no-airflow, not just raw over-temp.
- **Board adds:** pitot power stage (3 A MOSFET + gate drive + HW cutoff + reverse/TVS); **12 V input upsized** to carry pitot 3 A + pocket (Micro-Fit, not JST-GH); switched-12 V-out to the probe element (fold into one probe connector with the pitot RTD).
- **Cautions:** 36 W is dissipated in the remote element — the board only carries the MOSFET loss (~0.2–0.5 W) + 3 A traces; route wide copper, keep the pitot stage away from the isothermal pocket + precision analog, slow PWM for low EMI. Couples pitot anti-ice to this board (single-point failure) — accepted, HW cutoff covers stuck-on.
- **Resolves:** open Q2. **Amends:** REQ-07. **Traces to:** REQ-07, REQ-05, REQ-08, DEC-05.

### DEC-30 — Ethernet magnetics: discrete transformer + Bob Smith + ESD (not transformerless)
- **Decision:** Isolated **discrete Ethernet transformer** on-board (no transformerless option). Add **Bob Smith termination** (75 Ω to a chassis-ref node via cap) and a **low-cap Ethernet TVS/ESD array** on the four cable lines.
- **Why:** Ethernet may be routed anywhere across the airframe (~2 m). Over that distance the two ends can sit at different ground potentials (structure/ESC return currents) and a 2 m unshielded run picks up switching surges — transformerless has nothing to absorb either and would risk the PHY. Isolation >> the gram saved.
- **Impact:** minor — Rev E/G already carry the transformer block; this adds the termination network + TVS, no topology change. Drops the transformerless alternative.
- **Traces to:** REQ-04, DEC-26.

### SUPERSEDED

- **DEC-25/26/27 MCU part.** C092→C593→F407→**H563** (DEC-28); CAN-FD dropped at F407, **restored at H563**. Ethernet + dual-connector + LM66100 diode-OR architecture stands.
- **DEC-27 — F407 (classic CAN).** *Superseded by DEC-28* — H563 is in stock with FDCAN, so CAN-FD is back.
- **DEC-25 part choice (C092CB).** Architecture (MCU + CAN) stands; the *part* is superseded by DEC-26 — Ethernet requirement forces the C593.
- **DEC-21 — No on-board MCU.** *Superseded by DEC-25* (MCU added for CAN + autonomous heater).
- **DEC-22 — LTC4316 + host I²C/SPI buses.** *Superseded by DEC-25* (MCU dual-I²C separates the ILPS → LTC4316 gone; interface now CAN).
- **DEC-23 — Host-switched heater.** *Control superseded by DEC-25* (MCU PID, autonomous); heater element/MOSFET + HW over-temp backstop retained.
- **DEC-03 / DEC-04** — early differential + MS5611-static passes. *See DEC-07, DEC-13, DEC-14, DEC-15.*
- **DEC-07 — Dual ILPS28QSW, subtraction.** Superseded by DEC-13, then **re-adopted by DEC-15** (accepting lower accuracy).
- **DEC-08 — 2-ch I²C switch (PCA9540B).** Superseded by DEC-13/14, re-activated by DEC-15, superseded by DEC-17, then **re-opened by DEC-21** (no MCU → address separation via dual-bus/translator/switch again).
- **DEC-17 — Local MCU (self-contained compute).** *Superseded by DEC-21* — flight computer does the math; board stays dumb (sensors + front-ends + autonomous heater).
- **DEC-13 — Dedicated differential sensor; abandon subtraction.** *Superseded by DEC-15* (back to subtraction for manufacturability).
- **DEC-14 — Honeywell differential + MS5611 static.** *Superseded by DEC-15* (Honeywell availability too low). MS5611-for-static portion **re-adopted by DEC-16**.

---

## Open questions (burn down → promote to DEC)

- [ ] **Measure actual die-to-die gain mismatch** (10-min two-point bench check, both teed to one source) → pins the airspeed accuracy within the 0.05–0.2 % band. (DEC-15)
- [ ] **Confirm ARK/flight-computer speaks CAN-FD** to realize FD data rates (else the node runs classic on the FD-capable HW). (DEC-25)
- [ ] **Custom 6-pin Eth cable** — maps node TX±/RX±/5 V/GND to ARK 4-pin data + separate power. (DEC-26)
- [ ] **H563 package** — LQFP64 (RIT6) vs LQFP100 (VIT6): confirm RMII + 2×I²C + SPI + FDCAN + heater pins fit in CubeMX. (DEC-28)
- [ ] **Crystal** — 8–16 MHz for CAN bit timing (cold); confirm value. (DEC-25)
- [ ] **Pitot heater routing** — 12 V/3 A through this unit (size connector/traces) or stays on the power board? (DEC-19)
- [ ] **Machined-case manifold** — 3 sealed O-ring ports (2× ILPS + MS5611), pitot/static barbs, condensate drainage. (DEC-20)
- [ ] **Pocket thermal isolation + heater budget** — setpoint, watts, insulation island design. (DEC-19/20)
- [ ] **Connector finalization** — data+5 V (JST-GH), 12 V (Micro-Fit), 2× RTD, 2 barbs. (DEC-17/18/19)
- [ ] **Does the PAB already pull up SDA/SCL?** If so, host-side pull-ups DNP.
- [ ] **Airframe supply to the power board** — 12 V present or converter needed for 36 W heater? (REQ-07)
- [ ] **Autopilot / flight controller identity** — affects host interface choice. (DEC-17)

---

## Revision history

| Rev | Date | Summary |
|-----|------|---------|
| A | superseded | Dual-ILPS28QSW board: 5 V→3.3 V LDO, 2-ch I²C switch, per-channel pull-ups. Matched DEC-15; superseded by the self-contained unit (DEC-16…20). |
| B | (dropped) | MS5611 + Honeywell differential — abandoned (DEC-15). |
| C | superseded | No-MCU sensing unit (LTC4316 + host I²C/SPI). `airdata_rev_c.kicad_sch`. Superseded by Rev D (DEC-25). |
| D | superseded | CAN-only node (STM32C092, single connector). `airdata_rev_d.kicad_sch`. Superseded by Rev E (DEC-26). |
| E | superseded | Same as F but with a Schottky diode-OR power input. `airdata_rev_e.kicad_sch`. Superseded by Rev F (ideal-diode mux). |
| F | superseded | C593 + TPS2116 mux, single heater. `airdata_rev_f.kicad_sch`. Superseded by Rev G. |
| G | **built** | **STM32H563** + 2× ILPS28QSW (I2C1/I2C2) + MS5611 + 2× MAX31865 (SPI); **CAN-FD** (SIT1044) 4p + 100BASE-TX (LAN8742 + transformer + ESD/Bob-Smith) 6p; both 5 V via 2× **LM66100** ideal-diode OR; **pocket + pitot (3 A) heaters** (MCU PWM, HW cutoffs); 12 V/3 A in (Micro-Fit); combined pitot probe connector (RTD+heater); AP2112K LDO; SWD + 2 crystals. `airdata_rev_g.kicad_sch` (functional draft — CubeMX pins, real vendor symbols, verify PHY/magnetics/pitot-backstop, footprints, ERC). |
