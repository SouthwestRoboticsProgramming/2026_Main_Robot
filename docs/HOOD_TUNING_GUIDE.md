# Hood Auto-Aim Tuning Guide

This guide walks through how to tune the adjustable hood so it automatically sets the correct angle based on distance to the target. The system uses an interpolation table — you teach it the correct hood angle at several known distances, and it smoothly blends between them.

## What You Need

- Robot deployed with latest code
- Shuffleboard or SmartDashboard connected
- Game pieces to shoot
- Tape measure (or use known field markings)
- Clear line of sight to AprilTags (for vision-based distance)

## Shuffleboard Layout Setup

Before tuning, add these key entries to your Shuffleboard layout:

**Must have (drag these from NetworkTables):**
- `Shooter/DistanceToHub` — live distance to target in meters
- `Shooter/Hood Actual (deg)` — current hood angle from encoder
- `Shooter/Hood Mode` — current mode (MANUAL, AUTO_TRACK, etc.)
- `ShooterMech2d` — visual widget showing hood angle + distance bar

**Tuning controls:**
- `Shooter/Aim/Tuning/Lock Distance` — checkbox, freezes distance reading
- `Shooter/Aim/Tuning/Save Shot` — button, saves current shot data
- `Shooter/Aim/Tuning/Next Save Slot` — shows which slot will be written next (0-5)
- `Shooter/Hood/Nudge Step (deg)` — how many degrees each nudge adjusts (default 0.5)

**Interpolation table (auto-populated, you'll see values update as you save):**
- `Shooter/Aim/Hood Point 0-5 Dist (m)`
- `Shooter/Aim/Hood Point 0-5 Angle (deg)`
- `Shooter/Aim/RPS Point 0-5 Dist (m)`
- `Shooter/Aim/RPS Point 0-5 RPS`

**Fine tuning:**
- `Shooter/Aim/Hood Angle Offset (deg)` — shifts ALL angles up/down by a fixed amount
- `Shooter/Aim/Base Flight Time (s)` — for shoot-on-the-move compensation
- `Shooter/Aim/Flight Time Per Meter (s)` — for shoot-on-the-move compensation

## Step-by-Step Tuning Procedure

### Phase 1: Collect Data Points (The Main Tuning Loop)

You have 6 slots (Point 0 through Point 5). Start close to the target and work outward.

**For each distance:**

1. **Position the robot** at a known distance from the target. Use a tape measure or known field markings. Start at your closest expected shooting distance (~1m) and work out to your farthest (~3.5m).

2. **Verify the distance reading.** Check `Shooter/DistanceToHub` in Shuffleboard. Make sure AprilTags are visible and the number looks reasonable.

3. **Lock the distance.** Toggle `Shooter/Aim/Tuning/Lock Distance` to ON. This freezes the distance value so vision jitter doesn't shift it while you tune. You'll see `Shooter/DistanceToHub` stay fixed.

4. **Switch hood to MANUAL mode.** Use the controller or set `Shooter/Hood Mode` so you have direct control of the hood angle.

5. **Adjust the hood angle.** Use the nudge up/down buttons on the controller to move the hood in small increments. Watch `Shooter/Hood Actual (deg)` in Shuffleboard. The step size is controlled by `Shooter/Hood/Nudge Step (deg)`:
   - Start at **2.0 deg** to get in the ballpark quickly
   - Switch to **0.5 deg** (default) for fine-tuning once you're close

6. **Shoot and observe.** Fire test shots. Adjust the hood angle until shots consistently hit the target. Take your time here — this is the most important step.

7. **Save the shot.** Once shots are landing consistently, toggle `Shooter/Aim/Tuning/Save Shot` to ON. This automatically records:
   - The current locked distance into the hood and RPS distance fields
   - The actual hood encoder angle into the hood angle field
   - The current shooter RPS into the RPS field

   The `Next Save Slot` counter advances automatically.

8. **Unlock the distance.** Toggle `Shooter/Aim/Tuning/Lock Distance` back to OFF.

9. **Move to the next distance** and repeat from step 1.

### Suggested Distances

For 6 data points, a good spread is:

| Slot | Distance | Notes |
|------|----------|-------|
| 0 | ~1.0 m | Closest shot (near fender) |
| 1 | ~1.5 m | Close-mid range |
| 2 | ~2.0 m | Mid range |
| 3 | ~2.5 m | Mid-far range |
| 4 | ~3.0 m | Far range |
| 5 | ~3.5 m | Maximum shooting distance |

You don't need to be at these exact distances. The system interpolates between whatever distances you record.

### Phase 2: Test Auto-Tracking

1. **Switch hood to AUTO_TRACK mode.** The hood will now automatically adjust based on distance.

2. **Drive around at various distances** and shoot. The interpolation table fills in angles between your recorded points.

3. **Check the edges.** Pay special attention to distances at or beyond your closest and farthest recorded points. The system clamps to the nearest endpoint outside the table range. The `Shooter/Aim/In Range` indicator tells you whether you're within the table bounds.

### Phase 3: Competition-Day Quick Adjust

If shots are consistently off at competition (different game pieces, field conditions, etc.), use the **Hood Angle Offset** instead of re-tuning every point:

- `Shooter/Aim/Hood Angle Offset (deg)` shifts every calculated angle by a fixed amount
- If shots are going high, decrease the offset (negative)
- If shots are going low, increase the offset (positive)
- This is one number that fixes everything — much faster than re-tuning individual points

### Phase 4: Shooter RPS Tuning (Optional)

The RPS (flywheel speed) table works the same way as the hood angle table. The Save Shot button records both simultaneously. If you want to tune RPS independently:

1. Edit `Shooter/Aim/RPS Point X Dist (m)` and `Shooter/Aim/RPS Point X RPS` directly in Shuffleboard
2. Higher distances may need higher RPS to maintain shot accuracy
3. All RPS values are in rotations per second of the flywheel

## Troubleshooting

| Problem | Check |
|---------|-------|
| Distance reading is 0 or jumping wildly | No AprilTags visible to Limelights. Reposition robot so cameras can see tags. |
| Hood doesn't move in MANUAL | Is it still in HOMING mode? Wait for homing to complete (current spike detection). |
| Save Shot doesn't seem to work | Check `Next Save Slot` — it may have wrapped to 0 and overwritten your first point. |
| AUTO_TRACK angle seems wrong | Check `Shooter/Aim/In Range`. If false, you're outside your recorded distances. |
| Shots were good in practice but off at comp | Use `Hood Angle Offset (deg)` — one knob to shift everything. |
| Hood overshoots or oscillates | Tune PID at `Shooter/Hood/PID` — try reducing kP or increasing kD. |
| Need more than 6 data points | Change `NUM_POINTS` in `AimCalc.java` and redeploy. |

## All Tunable Parameters Reference

| NetworkTables Path | Default | Purpose |
|---|---|---|
| `Shooter/Hood/Nudge Step (deg)` | 0.5 | Degrees per nudge button press |
| `Shooter/Hood/Max Angle (Deg)` | 40.0 | Soft upper limit for hood angle |
| `Shooter/Hood/Min Angle (Deg)` | 22.7 | Soft lower limit for hood angle |
| `Shooter/Hood/PID` | kP=6.0, kI=0.8 | Hood position PID gains |
| `Shooter/Aim/Hood Angle Offset (deg)` | 0.0 | Global angle shift (competition quick-fix) |
| `Shooter/Aim/Base Flight Time (s)` | 0.4 | Shoot-on-the-move: base projectile travel time |
| `Shooter/Aim/Flight Time Per Meter (s)` | 0.10 | Shoot-on-the-move: additional time per meter |
| `Shooter/Aim/Hood Point X Dist (m)` | varies | Interpolation table distance entries |
| `Shooter/Aim/Hood Point X Angle (deg)` | varies | Interpolation table angle entries |
| `Shooter/Aim/RPS Point X Dist (m)` | varies | Interpolation table RPS distance entries |
| `Shooter/Aim/RPS Point X RPS` | varies | Interpolation table RPS entries |
| `Shooter/Aim/Tuning/Lock Distance` | false | Freeze distance for stable tuning |
| `Shooter/Aim/Tuning/Save Shot` | false | One-click save current shot data |
| `Shooter/Aim/Tuning/Next Save Slot` | 0 | Which slot Save Shot writes to next |

All values marked `.setPersistent()` survive robot reboots — you only need to tune once.
