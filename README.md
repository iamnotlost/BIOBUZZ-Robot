# BIOBUZZ Robot

Robot code for the 2026–2027 FIRST Tech Challenge season, BIOBUZZ™ presented by RTX.

Built on the FTC SDK with Pedro Pathing for autonomous paths and goBILDA Pinpoint odometry.

Every device is optional. If something isn't in the robot configuration, the code skips it
instead of crashing, and the Driver Station shows it on the **Missing** line.

## Robot configuration names

| Name | Device | Used for |
|---|---|---|
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | Motor | Mecanum drive |
| `pinpoint` | goBILDA Pinpoint | Odometry and heading (falls back to `imu`) |
| `imu` | Control Hub IMU | Heading if there's no Pinpoint |
| `intake` | Motor | Intake |
| `shooter`, `secondShooter` | Motor | Launcher (second motor follows the first) |
| `supply` | Motor | Feeds balls into the launcher |
| `button` | Digital touch sensor | Ball in position for launching |
| `colorSensor` | REV Color Sensor V3 | Tells POLLEN, red NECTAR and blue NECTAR apart |

## OpModes

**Testing (do these first on a new robot)**
- `hardwareTest` – runs any motor or servo in the configuration, whatever it's named. D-pad up/down picks a device, left stick runs it. Also shows heading, Pinpoint position and the color sensor reading.
- `driveTest` – drive base only. Hold a D-pad direction to spin a single wheel (up = frontLeft, right = frontRight, down = backRight, left = backLeft); each one should push the robot forward. Square switches field-/robot-centric.

**Setup**
- `setupParkRed`, `setupParkBlue` – teach the safe park auto: SQUARE on the corner start spot, drive into the LOADING ZONE, O saves.
- `setupAutoRed`, `setupAutoBlue` – teach the shooting auto: drive the robot to its start spot and press SQUARE, drive to where it shoots and press X, drive partly into the LOADING ZONE and press O. Hold triangle test-shoots.
- `settingsMenu` – change shot speeds, intake, drive speed, auto park distances and more with the D-pad, press X to save them on the robot (file `FIRST/biobuzz_settings.txt`). No re-upload needed. Nothing moves in this menu.

**TeleOp**
- `redTeleOp`, `blueTeleOp` – full driver control with the 2:00 match timer.

**Autonomous**
- `leaveAuto` – drives 12 in. off the wall: LEAVE, 3 pts.
- `parkRed`, `parkBlue` – safe park: start in our corner, drive straight off the wall, then sideways into the LOADING ZONE (LEAVE + PARK = 8 pts) without getting in the other alliance's way. Teach the park spot with `setupParkRed` / `setupParkBlue`.
- `shootRed`, `shootBlue` – drive to the taught shoot spot, shoot all 4 pre-loaded POLLEN, drive to the taught end spot (put it in the LOADING ZONE, off the wall: LEAVE + PARK). Teach the spots with `setupAutoRed` / `setupAutoBlue`. Two plans (A / B, different start spots) are picked with the D-pad during INIT. The Driver Hub shows each step and what happened; the same lines are in the robot log (tag `AUTO`).

## TeleOp controls (gamepad 1)

| Control | Action |
|---|---|
| Left stick | Drive. Robot-centric by default: forward = where the robot faces |
| Click left stick | Switch between robot-centric and field-centric (forward = away from you) |
| Right stick X | Turn |
| Either trigger | Slow down gradually: the harder you squeeze, the slower (down to 25%) |
| Options | Reset heading (point the robot away from you first) |
| Share | Heading hold on/off (off by default; switches itself off if the heading reading looks wrong) |
| Right bumper (R1) | **Hold** to load: intake only (slow); the feeder stays still so balls settle where it grips them |
| Cross | **Hold**: feeder pushes balls up |
| Circle | **Hold**: intake and feeder backwards (pull a ball back down / spit it out) |
| Triangle | **Hold** to shoot: the wheel spins up first; the feeder runs only while the wheel is at the shot speed and stops after each ball until the wheel is back at speed, so every ball flies the same. Let go to stop |
| Left bumper (L1) | **Hold** to pre-spin the shooter, so triangle fires sooner |
| D-pad up/down | Shot stronger/weaker by 10, saved on the robot at once (one controller is enough to tune shots) |
| Gamepad 2 D-pad left/right | Shot speed -/+ 10, saved at once |
| Gamepad 2 cross | SAVE all settings on the robot |
| Click right stick | Drive top speed 40% → 60% → 80% → 100% → back to 40% (buzzes 1–4 times), remembered on the robot |
| Gamepad 2 R1/L1 | Drive top speed up/down by 10%, remembered |
| Gamepad 2 D-pad up/down | Intake speed up/down (starts at 300) |

Driving is softened on purpose: small stick pushes move the robot gently, and it speeds up over about
half a second (by time, not loop speed) instead of jerking. Letting go of a stick stops the robot at once. Shot and drive speed changes are saved at once. Intake
speed changes are kept only after gamepad 2 cross (or saving in `settingsMenu`).

While shooting, the robot writes wheel speed, feeding and battery voltage to its log about 20 times a
second (tag `SHOT`), so a test session can be read back and the shot tuned from real numbers.

Gamepads rumble: 2 short buzzes at 1:00 left (NECTAR may go into FLOWERS), one long buzz at 0:15
left (go park in the LOADING ZONE), and a short buzz when the shooter is ready (L1) or starts
shooting. The robot stops at 0:00. A long rumble means the color sensor sees the other alliance's NECTAR – spit it out.
For practice longer than 2:00, set `matchTimer.ENFORCE_MATCH_END = false`.

## Code layout
All team code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

- `hwManager` – hardware map and device names
- `drivetrain`, `intake`, `shooter`, `supply`, `elementSensor` – subsystems
- `baseTeleOp` – driver control, shared by `redTeleOp` and `blueTeleOp`
- `baseAuto`, `baseParkAuto` – autonomous helpers
- `matchTimer` – TELEOP clock
- `pedroPathing/` – path following constants and tuning
