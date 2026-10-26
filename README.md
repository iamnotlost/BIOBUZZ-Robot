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

**TeleOp**
- `redTeleOp`, `blueTeleOp` – full driver control with the 2:00 match timer.

**Autonomous**
- `leaveAuto` – drives 12 in. off the wall: LEAVE, 3 pts.
- `parkRed`, `parkBlue` – LEAVE, then park in the LOADING ZONE: 8 pts. The park distances are placeholders; measure on a real field and set them in `baseParkAuto`.

## TeleOp controls (gamepad 1)

| Control | Action |
|---|---|
| Left stick | Drive (field-centric) |
| Right stick X | Turn |
| Right / left trigger | Half / quarter speed |
| Options | Reset heading (point the robot away from you first) |
| Right bumper | Intake on/off |
| Circle | Intake reverse |
| Cross | Feeder on/off |
| Left bumper | Launcher on/off |
| Triangle / square | Launch everything (close / far) |
| D-pad left/right | Turn to a preset angle |

Gamepads rumble at 1:00 left (NECTAR may go into FLOWERS) and 0:20 left, and the robot stops
at 0:00. A long rumble means the color sensor sees the other alliance's NECTAR – spit it out.
For practice longer than 2:00, set `matchTimer.ENFORCE_MATCH_END = false`.

## Code layout
All team code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

- `hwManager` – hardware map and device names
- `drivetrain`, `intake`, `shooter`, `supply`, `elementSensor` – subsystems
- `baseTeleOp` – driver control, shared by `redTeleOp` and `blueTeleOp`
- `baseAuto`, `baseParkAuto` – autonomous helpers
- `matchTimer` – TELEOP clock
- `pedroPathing/` – path following constants and tuning
