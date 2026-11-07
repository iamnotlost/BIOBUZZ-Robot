package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

// Change robot settings at the field and save them on the Control Hub. Nothing moves here.
//   D-pad up/down     pick a setting          D-pad left/right  change it (hold to repeat)
//   Cross             SAVE                    Triangle          reset this setting to default
//   Circle            reload what is saved
@TeleOp(name="settingsMenu", group = "setup")
public class settingsMenu extends OpMode {
    private int selected = 0;
    private boolean prevUp, prevDown, prevCross, prevTriangle, prevCircle;
    private int prevChange = 0;
    private final ElapsedTime repeatTimer = new ElapsedTime();
    private double nextRepeat = 0.4;
    private String message = "";
    private boolean unsaved = false;

    @Override
    public void init() {
        message = settings.load() ? "Loaded saved settings" : "Could not read the settings file - using defaults";
    }

    @Override
    public void init_loop() {
        handleInput();
        show();
    }

    @Override
    public void loop() {
        handleInput();
        show();
    }

    private void handleInput() {
        int n = settings.ITEMS.size();
        if (gamepad1.dpad_up && !prevUp)     selected = (selected - 1 + n) % n;
        if (gamepad1.dpad_down && !prevDown) selected = (selected + 1) % n;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // left/right: one step per press; held down, it repeats after 0.4 s, then every 0.1 s
        int change = gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 : 0;
        boolean firstPress = change != 0 && change != prevChange;
        boolean repeat = change != 0 && change == prevChange && repeatTimer.seconds() > nextRepeat;
        if (firstPress || repeat) {
            settings.ITEMS.get(selected).change(change);
            unsaved = true;
            message = "";
            repeatTimer.reset();
            nextRepeat = firstPress ? 0.4 : 0.1;
        }
        prevChange = change;

        if (gamepad1.triangle && !prevTriangle) {
            settings.Item item = settings.ITEMS.get(selected);
            item.set(item.defaultValue);
            unsaved = true;
            message = item.label + " reset to default";
        }
        if (gamepad1.cross && !prevCross) {
            boolean ok = settings.save();
            unsaved = !ok;
            message = ok ? "SAVED on the robot" : "SAVE FAILED";
            gamepad1.rumble(ok ? 200 : 800);
        }
        if (gamepad1.circle && !prevCircle) {
            settings.load();
            unsaved = false;
            message = "Reloaded what is saved";
        }
        prevTriangle = gamepad1.triangle;
        prevCross = gamepad1.cross;
        prevCircle = gamepad1.circle;
    }

    private void show() {
        telemetry.addLine("D-pad up/down: pick   left/right: change   X: SAVE   Triangle: default   O: reload");
        telemetry.addLine(unsaved ? "* not saved yet - press X *" : message);
        telemetry.addLine();
        for (int i = 0; i < settings.ITEMS.size(); i++) {
            settings.Item item = settings.ITEMS.get(i);
            telemetry.addLine((i == selected ? ">> " : "   ") + item.label + ":  " + item.text());
        }
        telemetry.update();
    }
}
