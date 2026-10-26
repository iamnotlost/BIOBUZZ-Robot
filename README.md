# BIOBUZZ Robot

Robot code for the 2026–2027 FIRST Tech Challenge season, BIOBUZZ™ presented by RTX.

Built on the FTC SDK with Pedro Pathing for autonomous paths and goBILDA Pinpoint odometry.

## Layout
All team code is in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:

- `hwManager` – hardware map and motor setup
- `drivetrain`, `intake`, `shooter`, `supply` – subsystems
- `baseTeleOp` – driver control, shared by `redTeleOp` and `blueTeleOp`
- `matchTimer` – TELEOP clock with gamepad rumble at 1:00 and 0:20, stops the robot at 0:00
- `pedroPathing/` – path following constants and autonomous paths
