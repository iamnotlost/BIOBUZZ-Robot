package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Settings;

/**
 * Change robot settings at the field and save them on the Control Hub. Nothing moves here.
 * D-pad up/down pick, left/right change (hold to repeat), CROSS save, TRIANGLE default, CIRCLE reload.
 */
@TeleOp(name = "settingsMenu", group = "setup")
public class SettingsMenu extends OpMode {
    private static final int SHOWN = 12;   // about this many lines fit on the Driver Hub

    private int selected = 0;
    private int heldDirection = 0;
    private final ElapsedTime repeatTimer = new ElapsedTime();
    private double nextRepeat = 0.4;
    private String message = "";
    private boolean unsaved = false;

    @Override
    public void init() {
        message = Settings.load() ? "Loaded saved settings" : "Could not read the settings file - using defaults";
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
        int n = Settings.ITEMS.size();
        if (gamepad1.dpadUpWasPressed()) selected = (selected - 1 + n) % n;
        if (gamepad1.dpadDownWasPressed()) selected = (selected + 1) % n;

        // left/right: one step per press; held, it repeats after 0.4 s, then every 0.1 s
        int direction = gamepad1.dpad_right ? 1 : gamepad1.dpad_left ? -1 : 0;
        boolean firstPress = direction != 0 && direction != heldDirection;
        boolean repeat = direction != 0 && direction == heldDirection && repeatTimer.seconds() > nextRepeat;
        if (firstPress || repeat) {
            Settings.ITEMS.get(selected).change(direction);
            unsaved = true;
            message = "";
            repeatTimer.reset();
            nextRepeat = firstPress ? 0.4 : 0.1;
        }
        heldDirection = direction;

        if (gamepad1.triangleWasPressed()) {
            Settings.Item item = Settings.ITEMS.get(selected);
            item.set(item.defaultValue);
            unsaved = true;
            message = item.label + " reset to default";
        }
        if (gamepad1.crossWasPressed()) {
            boolean ok = Settings.save();
            unsaved = !ok;
            message = ok ? "SAVED on the robot" : "SAVE FAILED";
            gamepad1.rumble(ok ? 200 : 800);
        }
        if (gamepad1.circleWasPressed()) {
            Settings.load();
            unsaved = false;
            message = "Reloaded what is saved";
        }
    }

    private void show() {
        telemetry.addLine("D-pad up/down: pick   left/right: change   X: SAVE   Triangle: default   O: reload");
        telemetry.addLine(unsaved ? "* not saved yet - press X *" : message);
        telemetry.addLine();
        int n = Settings.ITEMS.size();
        int first = Math.max(0, Math.min(n - SHOWN, selected - SHOWN / 2));
        int last = Math.min(n, first + SHOWN);
        telemetry.addLine(String.format("(%d of %d)%s", selected + 1, n, first > 0 ? "   ^ more above" : ""));
        for (int i = first; i < last; i++) {
            Settings.Item item = Settings.ITEMS.get(i);
            telemetry.addLine((i == selected ? ">> " : "   ") + item.label + ":  " + item.text());
        }
        if (last < n) telemetry.addLine("   v more below");
        telemetry.update();
    }
}
