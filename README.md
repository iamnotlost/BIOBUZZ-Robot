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
- `settingsMenu` – change shot speeds, intake, drive speed, auto park distances and more with the D-pad, press X to save them on the robot (file `FIRST/biobuzz_settings.txt`). No re-upload needed. Nothing moves in this menu.

**TeleOp**
- `redTeleOp`, `blueTeleOp` – full driver control with the 2:00 match timer.

**Autonomous**
- `leaveAuto` – drives 12 in. off the wall: LEAVE, 3 pts.
- `parkRed`, `parkBlue` – LEAVE, then park in the LOADING ZONE: 8 pts. The park distances are placeholders; measure on a real field and set them in `baseParkAuto`.

## TeleOp controls (gamepad 1)

| Control | Action |
|---|---|
| Left stick | Drive. Robot-centric by default: forward = where the robot faces |
| Click left stick | Switch between robot-centric and field-centric (forward = away from you) |
| Right stick X | Turn |
| Either trigger | Slow down gradually: the harder you squeeze, the slower (down to 25%) |
| Options | Reset heading (point the robot away from you first) |
| Share | Heading hold on/off (off by default; switches itself off if the heading reading looks wrong) |
| Right bumper (R1) | Load on/off: intake only (slow); the feeder stays still so balls settle where it grips them |
| Cross | Feeder pushes balls up on/off |
| Circle | Intake and feeder backwards on/off (pull a ball back down / spit it out) |
| Triangle | Shoot (close) on/off: the wheel spins up first, then the feeder pushes balls in only while the wheel is at full speed |
| Square | Shoot (far) on/off, same as triangle |
| Left bumper (L1) | Pre-spin the shooter on/off, so triangle fires instantly |
| D-pad left/right | Pick the shot preset: CLOSE / MID / FAR (triangle shoots it, square always shoots FAR) |
| Gamepad 2 D-pad left/right | Selected shot speed -/+ 20 |
| Gamepad 2 cross | SAVE all settings on the robot |
| D-pad up/down | Drive top speed up/down (starts at 40%) |
| Gamepad 2 D-pad up/down | Intake speed up/down (starts at 300) |

Driving is softened on purpose: small stick pushes move the robot gently, and it speeds up over about
half a second (by time, not loop speed) instead of jerking. Speed changes made with the D-pad last until the robot app restarts;
to make one permanent, change `MAX_SPEED` in `drivetrain.java` or `IN_VELOCITY` in `intake.java`.

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
