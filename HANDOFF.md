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
- Ball buttons (R1, ✕, ○, △, L1) are **hold to run, let go to stop** (changed from tap on / tap off at the arena on 2026-11-07).

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
- **Shooter:** one big wheel in a curved hood, two motors (`shooter` + `secondShooter`). Speed control is our own
  (`shooter.java`): both motors get the SAME power = kF·target + kP·error + small learned extra (kF 0.00037, kP 0.001,
  kI 0.003 learning only while waiting for the first ball, kept between bursts; picked with a simulation fitted to
  the robot log. kF 0.00028 was too low (wheel stuck at 1330 of 1420); stronger kP or a full-power boost made it overshoot to
  1540 after every ball; correcting for battery voltage made it worse because the battery sags to ~8.5 V on spin-up).
  Speed is read from the `shooter` encoder. (The old setup — SDK velocity PID on one motor, the second copying a
  fixed power — made the motors fight and the wheel ran ~90 ticks/s above the target.)

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
| `drivetrain` | Mecanum math. Robot-centric and field-centric. Left stick read as one arrow (radial dead zone 0.06, no sideways drift); `shape()` = any push past the dead zone starts at `MIN_DRIVE_POWER` 0.07 (turn 0.06) and curves (^1.8) up to top speed; triggers lower only the top speed. Speeds up in `RAMP_SECONDS` 0.2, slows/stops in `STOP_SECONDS` 0.1 (brake). Tuned 2026-11-12 after "slow to react, small moves dead, jerky stops, too fast"; `MAX_SPEED` 0.4, `TURN_SPEED` 0.4 |
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
| **R1** | **Hold** to load: intake pulls balls in; the feeder stays **still** so balls settle where it grips them |
| **✕** | **Hold**: feeder pushes balls up |
| **○** | **Hold**: intake + feeder backwards (spit out / pull ball back) |
| **△** | **Hold** to shoot at the one **shot speed**, let go to stop |
| **D-pad up/down** | Shot **stronger/weaker by 10**, saved on the robot at once (tune with one controller) |
| L1 | **Hold** to pre-spin the shooter at the shot speed |
| Gamepad 2 D-pad up/down | Intake speed ±100 |
| Gamepad 2 D-pad left/right | Shot speed ±10, saved at once |
| **Click right stick** | Drive top speed 40 → 60 → 80 → 100% → 40% (buzzes 1–4 times), saved at once |
| Gamepad 2 R1 / L1 | Drive top speed ±10%, saved at once |
| Gamepad 2 **✕** | **Save all settings** on the robot |

**How shooting works (hold △):** optional back-off (`BACK_OFF_SECONDS`, now 0) → spin up → feed only when the wheel is
within `READY_TOLERANCE` (50) of target **and has stayed there `SETTLE_MS` (100 ms)** (hub `getVelocity` reading,
steps of 20). **Don't tighten this blindly:** on 2026-11-12 a ±20 window with an averaged speed made the feeder start
and stop, balls stuck against the wheel and shots came out weak or not at all; it was reverted to this version, which
shot into the net every time at 1240. → after each ball the wheel
drops; feeding pauses until it settles again. Then the balls go **one after another**: the first ball waits for the wheel to be at speed and steady; after each ball
the wheel control brings it back without overshooting and, with `WAIT_BETWEEN_BALLS` (settingsMenu "Shot: wait for wheel
between balls", default ON), the feeder pauses only until the wheel is back within ±50 (no extra settle), so every
ball leaves at the same speed. OFF = feeder never stops after the first ball (fastest, later balls may fly lower). No "feed anyway" time limit; after 2 s of waiting the screen
says the wheel CAN'T REACH the speed (lower it or change battery). One shot speed for every shot (no CLOSE/MID/FAR):
`SHOT_VELOCITY` **1240** ticks/s (shot into the net every time, 2026-11-12); feeder stops when the wheel
drops `DROP_TOLERANCE` (80) below = a ball hit it; feeder while shooting 1500; ✕ feeder 1400.

**Driver screen** shows: time (+ "GO PARK" / "NECTAR allowed"), shooter status (+ balls shot), ball status, shot speed,
intake speed, battery, driving mode, heading hold, warnings (Pinpoint lost, opponent NECTAR, missing devices).

**Shot recorder:** while shooting, the robot logs a line every ~50 ms with tag `SHOT`:
`seconds,target,wheelSpeed,feeding(1/0),batteryVolts,motorPower` – read it from the robot log to tune shots from
real data.

## 6. Settings saved on the robot

File on the Control Hub: **`/sdcard/FIRST/biobuzz_settings.txt`** (Java Properties). Loaded at INIT of every
TeleOp and Auto. Change with the **`settingsMenu`** OpMode (D-pad up/down pick, left/right change, ✕ save,
△ default, ○ reload), gamepad 1 D-pad up/down (shot speed) or gamepad 2 in TeleOp. Items: shot speed, feeder speed when shooting, shot
ready window, back-off time, intake speed, ✕ feeder speed, drive top speed, turn speed, speed-up time, field-centric
at start, heading hold at start, auto start delay, auto leave distance, auto park forward / sideways (inches).

To read it from a computer: `adb shell cat /sdcard/FIRST/biobuzz_settings.txt`. When good values are found at the
field, copy them into the code defaults.

## 7. Autonomous

- `leaveAuto`: drive `LEAVE_DISTANCE` (12″) straight off the wall → LEAVE 3 pts.
- **`parkRed` / `parkBlue` = SAFE PARK** (when the opponents are strong and we stay out of their way): start in our
  CORNER touching the wall, drive straight off the wall (at least `MIN_OFF_WALL` 3″, forwards or backwards: the same way as the taught park spot, because the robot may start facing the wall), then sideways into our LOADING
  ZONE → LEAVE 3 + PARK 5. The park spot is taught with **`setupParkRed` / `setupParkBlue`** (INIT on the corner
  start, drive into the LOADING ZONE without touching the wall, ○ saves; keys redParkX/Y/H, bluePark…). If never
  taught, it falls back to the old `settingsMenu` numbers (leave distance, park forward/sideways; blue mirrors).
- Paths are **relative to the start pose** (0,0,0): +x = straight ahead, +y = robot's left. The robot may start facing the wall; then driving into the field is NEGATIVE x (backwards), which is fine.
- **Paths don't hold at the end** (`followPath(path, false)`): Pedro's automatic hold made the robot rock forward
  and backward at the spot. A drive also counts as done within `ARRIVED_IN` 1″ / `ARRIVED_DEG` 3° of the spot.
- **Shooter learning only when steady** (`shooter.STEADY_BAND` 20 over 0.1 s, and not while feeding): learning
  while the wheel was still speeding up ("windup") made it hold 20-30 above the target and creep higher every burst
  (1190 → 1210 → 1230 → 1240+, seen in the 2026-11-13 log). Simulated on the logged wheel: 1187-1194 every burst.
- **Shooter gentle start** (`shooter.SPINUP_RAMP_PER_S` 2.0 while below 80 % of target): both motors at 100 % from
  standstill dropped the battery to ~7 V and the Expansion Hub (shooter + feeder) cut out. If the Driver Hub still
  shows big drops, check the battery (try another), the battery connector and the main power switch.
- **Plan A / Plan B** (shooting auto): two taught routes per alliance with different START spots, chosen with the
  D-pad (left = A, right = B) during INIT of `shootRed/Blue` and `setupAutoRed/Blue`. For when the alliance partner
  needs our usual start spot (the other alliance can't take it: G304 = everyone starts on their own side). A plan is
  "set up" once its END spot is saved. Saving the SHOOT spot at the start (0,0,0) = shoot without driving first.
- **Debug:** every auto shows the current STEP, where the robot is, where it is going (inches to go) and the last 7
  EVENTS on the Driver Hub (reached X in 1.2 s, 0.8 in off / GAVE UP - blocked?, wheel ready at 1238, ball 2 hit the
  wheel, shooting stopped: all 4 balls / no ball for 1.5 s / time up, DONE at …). The same lines go to the robot log
  with tag `AUTO` (`adb pull /sdcard/robotControllerLog.txt`, search `AUTO`). INIT screen checks: plan set up, battery
  (warns < 12.5 V), missing devices, odometry (must stay 0,0,0 while still). The summary stays on screen after the
  auto until STOP.
- Every auto stops `SAFETY_MARGIN_S` (1 s) before 30 s.
- **`shootRed` / `shootBlue`** (main auto): START → drive to the taught SHOOT spot (turning to aim) → shoot all 4
  preloads with the same burst as TeleOp (`burst.java`; the upward CELL already has 3 NECTAR, so 4 POLLEN can TIP
  = 20 pts) → drive to the taught END spot. Stops shooting after 4 balls, 1.5 s without a ball, 8 s, or with 5 s
  of AUTO left. `preselectTeleOp` picks red/blueTeleOp afterwards.
- **`setupAutoRed` / `setupAutoBlue`** (teach mode, TeleOp list, group setup): INIT anywhere (keep still), ▶,
  drive by hand (half speed, trigger = quarter). **□ = START here** (robot on its start spot touching the wall),
  **✕ = SHOOT here** (✕ right at the start = shoot without driving), **○ = LOADING ZONE here** (end spot, partly in
  it, not touching the wall); hold △ = test burst, hold R1 = load. Spots are saved measured from where □ was
  pressed (before □ existed, the start was the INIT spot, and teaching after INIT elsewhere saved a shoot spot 103″
  to the side). Screen: checklist, position from START, odometry check, warning if a spot is > 7 ft from START.
  Saved per alliance and plan in the settings file (`autoSpots.java`), editable in `settingsMenu`.
- **Safety net:** an auto never drives to a spot more than `baseAuto.MAX_SPOT_IN` (165″, the most possible on our half; 100 blocked a real 115″ LOADING ZONE spot) from the start; it logs
  "too far, NOT driving there" instead.
- **Rules (manual TU08):** G304 start touching the wall, own side, NOT in the LOADING ZONE, 4 POLLEN, motionless
  after INIT. LEAVE = not touching the wall and PARK = partly in own LOADING ZONE, both judged at the END of AUTO
  → the END spot must be in the LOADING ZONE and off the wall (returning to the start spot scores neither).
  G402 stay on own side in AUTO, G403 nothing powered in the transition, G417 only launch into the upward CELL.

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

1. **Arena tuning** (in progress): the one shot speed, auto park distances → then make them code defaults.
2. Check the recorded `SHOT` data: do all balls in a burst leave at the same speed? Tune `shooter.kF/kP/kI` / window / feed speed. The `SHOT` log now also has the motor power as a 6th column.
3. Teach and test `shootRed`/`shootBlue` at the field (setupAutoRed/Blue), check 4 POLLEN tip the HIVE.
4. FLOWER claw code once it's built (outtake servos already in config); enforce NECTAR only in last 60 s.
5. 4-ball limit (G407) and opponent-NECTAR rejection once a color/distance sensor is added.
6. D-pad turn-to-angle presets exist (`PRESET_TURNS_ENABLED = false`) – re-enable when HIVE angles are known.
