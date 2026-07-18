# Porting nano_stepper's Simple PID servo to SERVO57D

Source: Misfittech/nano_stepper (Mechaduino lineage) — the reference firmware
the MKS Servo boards descend from. Cloned to ~/src/nano_stepper. This is the
DEFAULT "works out of the box" servo mode, encoder-only, NO current sensing.
It's the mode we should have built first.

## Why our attempts locked/hunted

We drove a FIXED-lead voltage vector from raw `enc·pole_pairs`. Two fatal gaps:

1. **No encoder calibration.** nano_stepper ALWAYS reads position through a
   200-point calibration table: `y = calTable.fastReverseLookup(sampleAngle())`.
   A raw magnetic encoder is nonlinear; `enc·pole_pairs` drifts against the true
   rotor angle across a rev, so the field falls out of quadrature → lock/slip.
2. **Fixed lead instead of PID-driven lead.** The field lead is the CONTROL
   signal, not a constant. It saturates at ±one full step (±90° elec), always
   in the direction that reduces error → self-correcting, cannot lock.

## The algorithm (StepperCtrl::simpleFeedback, 6 kHz)

Units: mechanical angle is u16, ANGLE_STEPS = 65536/rev (same as our encoder).
fullStep = ANGLE_STEPS / fullStepsPerRotation = 65536/200 = 327.68 (50 pp motor).
CTRL_PID_SCALING = 1024.

    y     = calibrated_encoder_position + phase_prediction   // linearized + vel FF
    error = desired - y
    iTerm += Ki·error            // clamped to ±fullStep
    u     = Kp·error + iTerm + Kd·(error - lastError)   // clamped to ±fullStep
    ma    = |u|/fullStep · (currentMa - holdMa) + holdMa      // amplitude from |error|
            (clamped to currentMa)
    moveToAngle(y + u, ma)       // APPLY field at (position + u), amplitude ma
    lastError = error

Key: field angle = **position + u**. At error=0, field=position (holds at
holdMa). Under error, field leads by up to ±90° at up to currentMa. The lead and
the current both scale with error — a proportional servo, self-commutating.

## moveToAngle → our hardware

nano_stepper (A4954, current-DAC driver):
    a = (angle·fullStepsPerRotation·microsteps) / ANGLE_STEPS   // → sine steps
    dacSin = mA·|sin(a)| ; dacCos = mA·|cos(a)| ; bridge sign = sign(sin/cos)

Ours (SERVO57D, EG3013 voltage PWM bridge) — the equivalent is our existing
`apply_coil_voltages(va, vb)` with a signed sine/cos vector:
    theta_e = (angle · POLE_PAIRS)         // mechanical u16 → electrical u16 (wrap)
    va = amp · cos(theta_e)
    vb = amp · sin(theta_e)
    amp = ma scaled into our PWM full-scale (split_duty maps |v|·max_duty/i16::MAX)

So: `moveToAngle(y+u, ma)` ==
    theta_e = (y + u) · POLE_PAIRS
    apply_coil_voltages(amp·cos(theta_e), amp·sin(theta_e))   with amp ∝ ma.

We do NOT need the mechanical→microstep→sine chain; our sin/cos LUT takes the
electrical u16 directly. Direction: nano_stepper flips one of sin/cos sign for
reverse wiring (we saw the MT6816 counts reversed — same fix, negate encoder
term or the cos).

## Constants / defaults to port
- ANGLE_STEPS = 0x10000 (65536), matches encoder units.
- Control loop 6 kHz.
- fullStep = 327 (65536/200).
- PID scaling 1024. Tune sPID Kp/Ki/Kd via the matrix sweep (these are exactly
  the knobs to expose live). holdMa/maxMa also live knobs.

## Encoder calibration — the REAL algorithm (StepperCtrl::calibrateEncoder)

200-entry table, one per full step. `table[j]` = encoder value read at known
electrical step j. This is the missing piece — without it the field is only 90°
ahead over an ARC of the rev (motor spins when hand-helped through that arc, then
stalls/dithers in the dead zones — observed on hardware).

Capture routine (port faithfully, these details matter):

    microsteps = 1; feedback off; motorReset()
    steps = 0
    for j in 0..200:
        delay 200ms                       # let rotor SETTLE at this step
        mean = sampleMeanEncoder(200)      # AVERAGE 200 reads — encoder is noisy!
        table[j] = mean                    #   (single reads = our jitter all session)
        # advance ONE full step, but as TWO half-steps: a full-step jump can move
        # the rotor BACKWARD as current ramps between steps. Half-steps prevent it.
        for ii in 0..2:
            steps += A4954_NUM_MICROSTEPS/2
            driver.move(steps, currentMa)  # move the field vector
            delay 50ms
    # optional: smoothTable(); saveToFlash()

Runtime use — reverseLookup(encoderAngle) → true mechanical angle:
    find the two table entries table[i], table[i+1] that bracket the raw encoder
    reading (handle 65536 wrap), then INTERPOLATE:
        y = interp(table[i], i·65536/200,  table[i+1], (i+1)·65536/200,  enc)
    smooth linearization, not bucketed. Field then commutes 90° ahead EVERYWHERE.

Three details we must NOT skip (all are why our raw approach dithered):
  1. sampleMeanEncoder(200): average 200 reads/point. Single reads jitter.
  2. half-step advance: avoids backward jump during current ramp.
  3. settle delay before sampling each point.

Instrumentation: stream each point (j, mean, cal-distance) over CAN as it's
captured, so we WATCH the table build and can verify it's monotonic + one full
rev = 200 steps. Then testcal = max deviation from linear (should be < ~0.2°).

## Order
1. Task #67: encoder calibration table (drive slow open-loop through a rev,
   record raw-enc vs commanded, build reverse-lookup). THE missing foundation.
2. Task #68: simpleFeedback port above, gains + hold/max current as live OD
   knobs → matrix sweep. This is the working servo.
3. Current-based PID (mode 3) later, when we revisit current sense.
