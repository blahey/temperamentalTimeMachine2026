# Temperamental Time Machine — Planning Document

*Started: April 14, 2026*

---

## Project Vision

Four stepper motors, each moving a slide along a bass string. The position of each slide determines the pitch of that string. The firmware will manage musical intervals between all four strings and dynamically shift them to generate a musical outcome.

---

## Development Stages

### Stage 1 — Single motor, functional (current)
- Get one motor homing reliably and accepting serial angle commands
- Establish non-blocking, state-machine architecture that will scale to 4 motors

### Stage 2 — Mechanical calibration
- Map motor steps to slide position (mm) and pitch (Hz / musical note)
- Define usable range per string

### Stage 3 — Four-motor control
- Expand state machine for 4 independent motor instances
- Coordinate homing sequence for all four

### Stage 4 — Musical intelligence
- Define interval relationships between strings
- Implement dynamic shifting / musical pattern logic
- Decide on input/trigger mechanism (serial, MIDI, sensor, etc.)

---

## Coding Principles

- **Non-blocking throughout** — no `delay()` except in `setup()`, no `while` loops
- **Timing** — use `elapsedMillis` library for all elapsed-time tracking
- **State machine** — each motor managed via explicit state enum
- **Serial input** — non-blocking accumulation with newline delimiter

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Teensy 3.5 | PlatformIO, Arduino framework |
| Stepper Motor | NEMA 17 42BYGH4807 (CW-MOTOR) | 200 steps/rev, 4.2 kg-cm, 4-wire |
| Driver | A4988 | Step/Dir interface |
| Limit Switch | Pin 23 | External pull-up, active LOW |
| Potentiometer | Pin A13 | Reserved, currently unused |

**Motor Configuration**
- Microstepping: 8× → 1600 steps/rev
- Enable pin: 16 (shared) | MS pins: 17, 18, 19 (shared)
- Motor 0: step=14, dir=15 | Motor 1: step=37, dir=38
- Motor 2: step=35, dir=36 | Motor 3: step=33, dir=34

**Mechanical Drive**
- Lead screw: 8 mm linear travel per revolution
- 1600 steps/rev ÷ 8 mm/rev = **200 steps/mm**
- Homing backoff: 10 mm = 2000 steps

---

## Firmware Architecture (Stage 1)

### State Machine

```
STATE_IDLE  ──startMotorTest()──►  STATE_TESTING
STATE_IDLE  ──startHoming()────►  STATE_HOMING
STATE_TESTING ──test done──────►  STATE_HOMING  (via startHoming())
STATE_HOMING  ──switch LOW─────►  STATE_HOMED
STATE_HOMING  ──max steps hit──►  STATE_IDLE    (error)
STATE_HOMED   ──serial cmd─────►  (stays HOMED, myStepper.moveTo() called)
```

### Key Functions

| Function | Description |
|---|---|
| `updateLimitSwitch()` | Debounces limit switch every loop; uses `elapsedMillis` |
| `startMotorTest()` | Optional startup run; sets STATE_TESTING, non-blocking |
| `updateMotorTest()` | Checks distanceToGo; transitions to homing when done |
| `startHoming()` | Begins slow move toward switch; sets STATE_HOMING |
| `updateHoming()` | Monitors switch; zeros position on trigger via `setCurrentPosition(0)` |
| `updateHomed()` | Reads serial angle input; calls `myStepper.moveTo()` |
| `recvWithEndMarker()` | Non-blocking serial accumulation; integer parsed on `\n` |

**Note:** `HOMING_DIR` constant (`+1` or `-1`) sets which direction the slide homes toward. Flip if the limit switch is at the opposite end of travel.

---

## Open Questions / To Do

### Stage 1
- [ ] Confirm `HOMING_DIR` — which direction does the motor need to move to reach the limit switch?
- [ ] Characterize slide travel: total steps from home to far end of string
- [ ] Verify A4988 Vref / current limit is set correctly for the motor
- [ ] Add enable/disable logic to cut motor current when idle (reduce heat)

### Stage 2+
- [ ] Define step-to-pitch mapping per string
- [ ] Decide on multi-motor architecture (array of structs or separate instances?)
- [ ] Define musical interval logic
- [ ] Choose input/trigger mechanism for musical control

---

## Reference Links

- [A4988 Tutorial — Last Minute Engineers](https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/)
- [AccelStepper Library Docs](https://www.airspayce.com/mikem/arduino/AccelStepper/index.html)
- [Pololu A4988 Product Page](https://www.pololu.com/product/1182)
- [How Stepper Motors Work (video)](https://youtu.be/TWMai3oirnM?si=IP0x5L8xtVg3shti)
- [Arduino Uno A4988 Circuit Demo (video)](https://youtu.be/sER5GNjcQ70?si=cHnAq-LED0kjjmdD)
- [Motor Datasheet — Circuit Specialists](https://www.circuitspecialists.com/nema_17_stepper_motor_42bygh4807)
- [Serial Input Basics (forum)](https://forum.arduino.cc/t/serial-input-basics-updated/382007/3)

---

## Notes / Log

**2026-04-14** — Created planning document. Firmware had basic stepper control, debounced limit switch reading, and several inactive motion modes. Homing routine was the next logical step.

**2026-04-14** — Rewrote firmware with non-blocking state machine. Changes:
- Added `elapsedMillis` for switch debounce (removed manual `millis()` tracking)
- Removed `while` loop from motor test; replaced with `STATE_TESTING` state
- Replaced old `limitSwitchStepTest()` with proper `startHoming()` / `updateHoming()` flow
- `myStepper.setCurrentPosition(0)` used on switch trigger to stop motor and zero position atomically
- Removed deprecated functions: `limitSwitchStepTest`, `runMotorTest` (blocking), `randStepper`, `variableStepper`, `angleInputNB`, `stepsInputNB`
- Angle control now lives in `updateHomed()` only (active when homed)
