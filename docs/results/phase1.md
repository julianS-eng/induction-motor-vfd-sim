#### Model validation summary

| Check | Result |
|---|---|
| No-load speed vs 120·f/P (45 Hz, frictionless) | 1,350.000 vs 1,350.000 r/min (error 8.8e-09) |
| dq torque vs equivalent circuit (12 locked-speed points) | max abs(ΔT) = 2.8e-07 N·m |
| dq current vs equivalent circuit | max relative error 1.9e-04 |
| Steady-state power balance (45 Hz, rated load) | relative residual -2.6e-07 |
| Energy conservation over a 4.5 s start + load step | relative residual -4.7e-07 |

#### Motor LabVolt-8221-22-200W-4p-208V-60Hz: rated operating point (equivalent circuit, 208 V / 60 Hz)

| Quantity | Value | Rating plate (nameplate) |
|---|---|---|
| Speed | 1,685.0 r/min | 1,685 r/min |
| Slip | 6.39 % | 6.39 % |
| Shaft power | 200.0 W | 200 W |
| Line current | 1.140 A | 1.14 A |
| Power factor | 0.730 | - |
| Efficiency (no core loss) | 66.7 % | - |
| No-load current | 0.925 A | - |
| Breakdown torque / slip | 2.267 N·m (2.00× rated) at s = 0.320 | - |
| Locked-rotor current / torque | 3.73 A / 1.587 N·m | - |

#### Torque capability under V/f (equivalent circuit, 12% boost, 285 V bus, SPWM)

| Frequency | Applied line voltage | Breakdown torque | vs rated |
|---|---|---|---|
| 15 Hz | 70.7 V | 1.080 N·m | 0.95× |
| 30 Hz | 116.5 V | 1.528 N·m | 1.35× |
| 45 Hz | 162.2 V | 1.924 N·m | 1.70× |
| 60 Hz | 174.5 V | 1.596 N·m | 1.41× |

The SPWM voltage limit is reached at 49.25 Hz: above it the drive can no longer hold V/f constant and the machine is under-fluxed.

#### V/f steady state at rated load torque

| f / reference (r/min) | Applied V (peak, phase) | Stator flux | Speed, no slip comp. (slip) | Current (dq / eq. circuit) | Speed with slip comp. (f_out) |
|---|---|---|---|---|---|
| 15 Hz / 450 | 57.7 V | 0.194 Wb | 5.4 (98.80 %) | 1.455 / 1.455 A | 6.4 (18.34 Hz) |
| 30 Hz / 900 | 95.1 V | 0.374 Wb | 776.0 (13.78 %) | 1.134 / 1.134 A | 895.7 (33.89 Hz) |
| 45 Hz / 1350 | 132.5 V | 0.387 Wb | 1,233.3 (8.64 %) | 1.136 / 1.136 A | 1,351.4 (48.91 Hz) |
| 60 Hz / 1800 | 142.5 V | 0.303 Wb | 1,602.8 (10.95 %) | 1.222 / 1.221 A | 1,602.8 (60.00 Hz) |

#### Power balance at 45 Hz, rated load (averages over 0.5 s of steady state)

| Term | Power |
|---|---|
| Electrical input P_in | 243.30 W |
| Stator copper loss | 79.96 W |
| Rotor copper loss | 14.12 W |
| Motor bearing friction | 2.84 W |
| Delivered to the load | 146.39 W |
| Residual P_in − Σ | -6.4e-05 W (-2.6e-07 of P_in) |
| Efficiency (no core loss) | 60.2 % |

#### V/f start to 1500 r/min (30 % rated load, +70 % load step at 3 s, slip compensation on)

| Metric | Value |
|---|---|
| Time to 95 % of reference speed | 1.572 s |
| Peak phase current | 1.719 A |
| Speed before / after load step | 1,507.1 / 1,486.1 r/min |
| Minimum speed after the step | 1,432.5 r/min |
| Energy balance residual (electrical / mechanical) | -4.7e-07 / -1.2e-10 (relative) |

#### Slip compensation at 900 r/min (30 Hz)

| Slip compensation | Speed at 20 % load (error) | Speed at 100 % load (error) |
|---|---|---|
| off | 883.2 r/min (-1.87 %) | 776.0 r/min (-13.78 %) |
| on | 907.6 r/min (0.85 %) | 895.7 r/min (-0.48 %) |

#### Voltage boost at low frequency (30 % rated load, slip compensation off)

| Boost | Speed at 2 Hz (sync 60) | Speed at 3 Hz (sync 90) | Speed at 5 Hz (sync 150) | Speed at 8 Hz (sync 240) | Stator flux at 2 Hz |
|---|---|---|---|---|---|
| 0% | 0.3 | 0.8 | 2.0 | 4.4 | 0.076 Wb |
| 6% | 2.6 | 4.5 | 67.0 | 193.8 | 0.211 Wb |
| 12% | 23.5 | 58.1 | 123.4 | 216.5 | 0.373 Wb |

Speeds in r/min from the dq simulation. Without boost the stator resistance drop leaves too little flux and the motor stalls (speed ≈ 0). For reference, the rated flux is about 0.450 Wb (V/ω at the rating plate).
