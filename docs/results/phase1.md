#### Model validation summary

| Check | Result |
|---|---|
| No-load speed vs 120·f/P (45 Hz, frictionless) | 1,350.000 vs 1,350.000 r/min (error 6.6e-09) |
| dq torque vs equivalent circuit (12 locked-speed points) | max abs(ΔT) = 1.3e-05 N·m |
| dq current vs equivalent circuit | max relative error 2.0e-04 |
| Steady-state power balance (45 Hz, rated load) | relative residual -8.6e-08 |
| Energy conservation over a 4.5 s start + load step | relative residual -1.6e-07 |

#### Assumed motor: rated operating point (equivalent circuit, 208 V / 60 Hz)

| Quantity | Value | Nameplate (assumed) |
|---|---|---|
| Speed | 1,670.2 r/min | 1,670 r/min |
| Slip | 7.21 % | 7.22 % |
| Shaft power | 175.0 W | 175 W |
| Line current | 1.103 A | 1.2 A |
| Power factor | 0.553 | - |
| Efficiency (no core loss) | 79.6 % | - |
| No-load current | 0.896 A | - |
| Breakdown torque / slip | 2.935 N·m (2.93× rated) at s = 0.457 | - |
| Locked-rotor current / torque | 3.90 A / 2.322 N·m | - |

#### Torque capability under V/f (equivalent circuit, 6% boost, 285 V bus, SPWM)

| Frequency | Applied line voltage | Breakdown torque | vs rated |
|---|---|---|---|
| 15 Hz | 61.4 V | 2.123 N·m | 2.12× |
| 30 Hz | 110.2 V | 2.602 N·m | 2.60× |
| 45 Hz | 159.1 V | 2.819 N·m | 2.82× |
| 60 Hz | 174.5 V | 2.066 N·m | 2.07× |

The SPWM voltage limit is reached at 49.75 Hz: above it the drive can no longer hold V/f constant and the machine is under-fluxed.

#### V/f steady state at rated load torque

| f / reference (r/min) | Applied V (peak, phase) | Stator flux | Speed, no slip comp. (slip) | Current (dq / eq. circuit) | Speed with slip comp. (f_out) |
|---|---|---|---|---|---|
| 15 Hz / 450 | 50.1 V | 0.468 Wb | 341.1 (24.21 %) | 1.118 / 1.118 A | 467.3 (19.37 Hz) |
| 30 Hz / 900 | 90.0 V | 0.447 Wb | 779.1 (13.43 %) | 1.104 / 1.104 A | 909.3 (34.40 Hz) |
| 45 Hz / 1350 | 129.9 V | 0.439 Wb | 1,223.7 (9.36 %) | 1.102 / 1.102 A | 1,355.6 (49.44 Hz) |
| 60 Hz / 1800 | 142.5 V | 0.360 Wb | 1,604.7 (10.85 %) | 1.102 / 1.101 A | 1,604.7 (60.00 Hz) |

#### Power balance at 45 Hz, rated load (averages over 0.5 s of steady state)

| Term | Power |
|---|---|
| Electrical input P_in | 170.06 W |
| Stator copper loss | 25.51 W |
| Rotor copper loss | 13.53 W |
| Motor bearing friction | 2.79 W |
| Delivered to the load | 128.23 W |
| Residual P_in − Σ | -1.5e-05 W (-8.6e-08 of P_in) |
| Efficiency (no core loss) | 75.4 % |

#### V/f start to 1500 r/min (30 % rated load, +70 % load step at 3 s, slip compensation on)

| Metric | Value |
|---|---|
| Time to 95 % of reference speed | 1.577 s |
| Peak phase current | 2.284 A |
| Speed before / after load step | 1,503.9 / 1,499.3 r/min |
| Minimum speed after the step | 1,431.1 r/min |
| Energy balance residual (electrical / mechanical) | -1.6e-07 / -9.3e-11 (relative) |

#### Slip compensation at 900 r/min (30 Hz)

| Slip compensation | Speed at 20 % load (error) | Speed at 100 % load (error) |
|---|---|---|
| off | 877.1 r/min (-2.54 %) | 779.1 r/min (-13.43 %) |
| on | 905.0 r/min (0.55 %) | 909.3 r/min (1.03 %) |

#### Voltage boost at low frequency (50 % rated load, slip compensation off)

| Boost | Speed at 2 Hz (sync 60) | Speed at 3 Hz (sync 90) | Speed at 5 Hz (sync 150) | Speed at 8 Hz (sync 240) | Stator flux at 2 Hz |
|---|---|---|---|---|---|
| 0% | 1.0 | 2.3 | 8.2 | 148.8 | 0.207 Wb |
| 3% | 4.4 | 18.1 | 90.2 | 183.1 | 0.392 Wb |
| 6% | 29.0 | 59.0 | 115.8 | 200.4 | 0.614 Wb |

Speeds in r/min from the dq simulation. Without boost the stator resistance drop leaves too little flux and the motor stalls (speed ≈ 0). The rated flux is about 0.450 Wb (V/ω at the rating plate): at very low frequency the boost over-fluxes the unsaturated model.
