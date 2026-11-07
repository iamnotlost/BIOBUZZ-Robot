# BIOBUZZ robot – full handoff note

Paste this into a new Claude session (or point it at this file) to continue working on the robot.
Everything below was true on 2026-11-07.

---

## 1. Who / what

- FIRST Tech Challenge team **25584**, season **2026-2027, BIOBUZZ™ presented by RTX**.
- I (the user) am the team's **coder**. I'm not deeply technical: explain simply, step by step, and keep
  answers short. A teammate is the "robot maker" (builds and fixes the hardware).
- Code repo: **https://github.com/iamnotlost/BIOBUZZ-Robot** (public, branch `main`, GitHub user `iamnotlost`).
- **Commits must show only me as author. Never add a `Co-Authored-By: Claude` line.** Commit as `iamnotlost`.
- Present the code as the team's own new codebase; don't mention it came from last season's code.
- Workflow I like: I describe what the robot does wrong → you change the code → build → upload to the robot
  over USB-C → I test. I usually reply "plugged in, upload" when the cable is in the Control Hub.
- Controls must be **tap on / tap off**, not hold-to-use (hold was too hard to control).

## 2. The game (what matters for code)

- Match: 30 s AUTO, 8 s transition, 2:00 TELEOP. Robot must be motionless during the transition and after
  the end (rules G403/G404).
- Balls ("SCORING ELEMENTS"): **POLLEN** = yellow, 7.1 cm (2.8″); **NECTAR** = red/blue, 9.1 cm (3.6″).
- **HIVE**: two tipping CELLS on a central frame per alliance. LAUNCH balls into the upward CELL; ~8 POLLEN
  (or ~3 POLLEN + 3 NECTAR) tips it. Each **TIP = 20 pts** (AUTO or TELEOP). After a tip the *other* CELL is up.
  AprilTags are on the underside of each CELL (facing down).
- **FLOWER**: tube, 4″ opening, 21.5″ high, place from the top only. NECTAR may go into a FLOWER **only in the
  last 60 s** (G410).
- **GARDEN**: 1 pt per ball. **LEAVE** (off the wall in AUTO) = 3. **PARK** = partly in own **LOADING ZONE** =
  5 in AUTO and 5 at end of TELEOP. **SWARM RP** = LEAVE+PARK ≥ 16. POLLINATOR RPs = 4 and 7 tips.
- Max **4 balls** controlled at once (G407). Don't control opponent NECTAR (G408).
- Start: robot touches the perimeter wall on its side (red columns A–C, blue D–F), holding **4 preloaded POLLEN**.
- Max 8 motors + 8 servos; robot size 18×24×29″ expanded.

## 3. The robot hardware

**Electronics:** REV **Control Hub** (Android 7.1.2) + **Expansion Hub** (module address 2) + a **Servo Hub**
(address 3). goBILDA **Pinpoint** odometry computer on the Control Hub's I2C. Driven with a REV gamepad
(PlayStation-style buttons) plugged into a REV **Driver Hub**.

**Active robot configuration on the Control Hub: `MagisMain`** (another saved one, `testing`, is not used).

| Name | Device | Where |
|---|---|---|
| `frontLeft` / `backLeft` / `backRight` / `frontRight` | goBILDA 5202 motors (mecanum drive) | Control Hub ports 0 / 1 / 2 / 3 |
| `pinpoint` | goBILDA Pinpoint | Control Hub I2C port 1 |
| `outtakeProximitySensor` | digital | Control Hub digital 3 (not used by code) |
| `intake` | REV HD Hex (UltraPlanetary) | Expansion Hub port 0 |
| `supply` (the **feeder** that pushes balls up to the shooter) | REV HD Hex | Expansion Hub port 1 |
| `secondShooter` | REV HD Hex | Expansion Hub port 2 |
| `shooter` | REV HD Hex (has the encoder used for speed) | Expansion Hub port 3 |
| `button` | digital touch sensor | Expansion Hub digital 0 (probably not fitted) |
| `corection` (CR servo), `intakeYawServo`, `intakePitchServo` | servos | Expansion Hub |
| `outtakeTiltLeft/RightServo`, `outtakeDepositorClaw/Yaw/PitchServo`, `intakeTiltServo` | servos | Servo Hub |

The servos are **placeholders only** – the FLOWER claw is **not built yet**. Code never moves them.
There is **no IMU** in `MagisMain` and **no color sensor** on the robot yet.

**Mechanisms:**
- **Drive:** mecanum, 4 goBILDA motors.
- **Intake:** wide front roller with rubber bands/strips, chain driven. Motor direction in code is
  `FORWARD` = pulls balls in. **Must run slow (~300 ticks/s):** at higher speeds the 200 mm rubber bands
  stretch and fly off.
- **Feeder (`supply`):** carries balls up into the shooter. Positive speed = up.
- **Shooter:** one big wheel in a curved hood, two motors (`shooter` + `secondShooter`; the second just copies
  the first one's power). Velocity control PIDF = (25, 0, 0, 12.8).

**Known hardware issues:**
1. **Pinpoint I2C freezes:** its I2C cable sometimes gets stuck (`I2C_TIMEOUT_SDA_STUCK` in the log); this used to
   freeze the whole robot. Code now detects it and drives on without it. Real fix: reseat/replace the Pinpoint
   cable, route it away from motor wires, update hub firmware with REV Hardware Client 2.
2. **Robot turning by itself:** the Pinpoint calibrates during INIT; if the robot is bumped then, its heading
   drifts and "heading hold" used to chase it in circles. Heading hold is now **off by default** and gives up past
   30°. Keep the robot still during INIT.
3. **Two balls at once jam** in the intake (mechanical). Code has auto-unjam (brief reverse).
4. Early tests used a soft 3D-printed ball that squeezed through gaps; real solid balls got stuck until the robot
   maker fixed the path. Re-check tuning with real balls.

## 4. The code (TeamCode/src/main/java/org/firstinspires/ftc/teamcode/)

Built on FTC SDK **11.1**, **Pedro Pathing 2.0.2** for autonomous paths. Every device is optional: missing ones
stay `null`, code skips them, and the Driver Hub shows them on a "Not found / Missing" line.

| File | What it does |
|---|---|
| `hwManager` | Finds all devices by name, motor directions, **bulk reads**, Pinpoint with stuck-bus protection (`pinpointFailed`), heading, battery voltage, intake/shooter velocity |
| `drivetrain` | Mecanum math. Robot-centric and field-centric. Stick dead zone + squared curve, time-based ramp (`RAMP_SECONDS`), ramps movement as one arrow, `MAX_SPEED` 0.4, `TURN_SPEED` 0.3 |
| `intake` | Intake motor, `IN_VELOCITY` 300 / `OUT_VELOCITY` -300 |
| `supply` | Feeder motor |
| `shooter` | Shooter motors, `spinTo(speed)` |
| `elementSensor` | Color sensor ball detection (POLLEN / red / blue NECTAR) – ready for when a `colorSensor` is added |
| `matchTimer` | 2:00 TeleOp clock; rumble at 1:00 (NECTAR → FLOWER allowed) and 0:15 (go park); stops robot at 0:00 |
| `baseTeleOp` | All driver control (see section 5). `redTeleOp` / `blueTeleOp` just pick the alliance |
| `settings` + `settingsMenu` | Settings saved on the robot (section 6) |
| `baseAuto`, `leaveAuto`, `baseParkAuto`, `parkRed`, `parkBlue` | Autonomous (section 7) |
| `hardwareTest` | Runs any motor/servo by selection to check wiring (servos only move after you touch the stick) |
| `driveTest` | Drive base only; hold D-pad to spin one wheel |
| `pedroPathing/Constants`, `Tuning` | Pedro Pathing tuning (from this same robot last season) |
| `Alliance` | RED/BLUE, used to mirror things |

## 5. TeleOp controls (gamepad 1 unless noted)

| Button | Action |
|---|---|
| Left stick | Drive. **Robot-centric** by default (forward = where robot faces) |
| Click left stick | Switch robot-centric / field-centric |
| Right stick X | Turn |
| Either trigger | Slow down gradually (full squeeze = 25%) |
| Options | Reset heading |
| Share | Heading hold on/off (off by default) |
| **R1** | Load on/off: intake pulls balls in; the feeder stays **still** so balls settle where it grips them |
| **✕** | Feeder pushes balls up, on/off |
| **○** | Intake + feeder backwards, on/off (spit out / pull ball back) |
| **△** | Shoot the **selected preset**, tap again to stop |
| **□** | Shoot **FAR**, tap again to stop |
| **D-pad left/right** | Pick shot preset **CLOSE / MID / FAR** |
| D-pad up/down | Drive top speed ±10% |
| L1 | Pre-spin the shooter at the selected preset |
| Gamepad 2 D-pad up/down | Intake speed ±100 |
| Gamepad 2 D-pad left/right | Selected shot speed ±20 |
| Gamepad 2 **✕** | **Save all settings** on the robot |

**How shooting works (△/□):** optional back-off (`BACK_OFF_SECONDS`, now 0) → spin up → feed only when the wheel is
within `READY_TOLERANCE` (50) of target **and has stayed there `SETTLE_MS` (100 ms)** → after each ball the wheel
drops; feeding pauses until it settles again. Safety time limits: `SPINUP_TIMEOUT_SECONDS` 1.5, `RECOVER_TIMEOUT_SECONDS` 1.0.
Default speeds (ticks/s): CLOSE 1320, MID 1590, FAR 1860; feeder while shooting 1500; ✕ feeder 1400.

**Driver screen** shows: time (+ "GO PARK" / "NECTAR allowed"), shooter status, ball status, shot preset + speed,
intake speed, battery, driving mode, heading hold, warnings (Pinpoint lost, opponent NECTAR, missing devices).

**Shot recorder:** while shooting, the robot logs a line every ~50 ms with tag `SHOT`:
`seconds,target,wheelSpeed,feeding(1/0),batteryVolts` – read it from the robot log to tune shots from real data.

## 6. Settings saved on the robot

File on the Control Hub: **`/sdcard/FIRST/biobuzz_settings.txt`** (Java Properties). Loaded at INIT of every
TeleOp and Auto. Change with the **`settingsMenu`** OpMode (D-pad up/down pick, left/right change, ✕ save,
△ default, ○ reload) or gamepad 2 in TeleOp. Items: shot speed CLOSE/MID/FAR, feeder speed when shooting, shot
ready window, back-off time, intake speed, ✕ feeder speed, drive top speed, turn speed, speed-up time, field-centric
at start, heading hold at start, auto start delay, auto leave distance, auto park forward / sideways (inches).

To read it from a computer: `adb shell cat /sdcard/FIRST/biobuzz_settings.txt`. When good values are found at the
field, copy them into the code defaults.

## 7. Autonomous

- `leaveAuto`: drive `LEAVE_DISTANCE` (12″) straight off the wall → LEAVE 3 pts.
- `parkRed` / `parkBlue`: leave, then drive to (`PARK_FORWARD`, `PARK_SIDE`) inches from the start → PARK
  in LOADING ZONE. **Distances are placeholders** (12, 24) – measure at the field and set in `settingsMenu`.
  Blue mirrors red's sideways distance.
- Paths are **relative to the start pose** (0,0,0): +x = straight ahead, +y = robot's left.
- Every auto stops `SAFETY_MARGIN_S` (1 s) before 30 s.
- **Not built yet:** shooting the 4 preloads into the HIVE in auto; a "teach mode" (drive the robot by hand,
  press ✕ to save waypoints, autos replay them) was proposed.

## 8. Building and uploading from a Mac

The Control Hub is an Android device; code is built as an app and installed over USB-C (or Wi-Fi).

1. Install **Android Studio** (it bundles a Java 21 runtime and installs the Android SDK). The project needs
   Android platform **34** and build-tools; Android Studio offers to install them on first sync.
   *Gradle 8.13 does not run on Java 24/25* – use Java 17 or 21 (Android Studio's bundled one is fine).
2. `git clone https://github.com/iamnotlost/BIOBUZZ-Robot.git`, open the folder in Android Studio, let it sync.
3. **Plug the USB-C cable into the Control Hub on the robot – not the Driver Hub** (easy mix-up). Robot battery
   on, solid green light. Close **REV Hardware Client** first (it holds the USB connection).
4. Upload: Android Studio ▶ Run with "Control Hub" selected, **or** in Terminal:
   ```
   ./gradlew :TeamCode:installDebug
   adb shell am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper
   ```
   (`installDebug` does not restart the robot app; the `am start` line does.) `adb` lives in
   `~/Library/Android/sdk/platform-tools/` on a Mac.
5. If the robot was just switched on, wait ~30 s before uploading or the connection drops mid-install.
6. Wi-Fi alternative: connect the Mac to the robot's Wi-Fi (`25584-RC`-style name; password readable in REV
   Hardware Client 2), then `adb connect 192.168.43.1:5555`.

**Reading logs:** `adb pull /sdcard/robotControllerLog.txt` (full log), `/sdcard/FIRST/matchlogs/` (one per OpMode
run). Useful searches: `SHOT` (shot recorder), `SDA_STUCK` (Pinpoint cable), `stuck in stop` (frozen OpMode).

## 9. What's next (open items)

1. **Arena tuning** (in progress): shot speeds per preset, auto park distances → then make them code defaults.
2. Check the recorded `SHOT` data: do all balls in a burst leave at the same speed? Tune PIDF / window / feed speed.
3. Autonomous that shoots the 4 preloads into the HIVE + parks (biggest points gain); or the teach mode.
4. FLOWER claw code once it's built (outtake servos already in config); enforce NECTAR only in last 60 s.
5. 4-ball limit (G407) and opponent-NECTAR rejection once a color/distance sensor is added.
6. D-pad turn-to-angle presets exist (`PRESET_TURNS_ENABLED = false`) – re-enable when HIVE angles are known.
