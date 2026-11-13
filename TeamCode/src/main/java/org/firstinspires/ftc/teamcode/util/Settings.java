package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.ParkAuto;
import org.firstinspires.ftc.teamcode.opmodes.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShotSequencer;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;

/**
 * Settings changed at the field (settingsMenu, D-pad in TeleOp, setupAuto) and saved on the Control Hub in
 * /sdcard/FIRST/biobuzz_settings.txt, so they survive restarts without uploading code. Each setting is a static
 * value in the class that uses it; {@link #load()} copies the file into them, {@link #save()} writes them out.
 * The keys never change, so settings saved by older code still load.
 */
public final class Settings {
    private Settings() { }

    public static final File FILE = new File(AppUtil.FIRST_FOLDER, "biobuzz_settings.txt");

    interface Getter { double get(); }
    interface Setter { void set(double v); }

    public static final class Item {
        public final String key, label;
        public final double step, min, max, defaultValue;
        final boolean whole;   // shown and stored without decimals
        private final Getter getter;
        private final Setter setter;

        Item(String key, String label, double step, double min, double max, boolean whole, Getter g, Setter s) {
            this.key = key;
            this.label = label;
            this.step = step;
            this.min = min;
            this.max = max;
            this.whole = whole;
            this.getter = g;
            this.setter = s;
            this.defaultValue = g.get();
        }

        public double get() { return getter.get(); }

        public void set(double v) {
            v = Math.max(min, Math.min(max, v));
            setter.set(whole ? Math.round(v) : Math.round(v * 100) / 100.0);
        }

        public void change(int steps) { set(get() + steps * step); }

        public String text() {
            if (min == 0 && max == 1 && step == 1) return get() >= 0.5 ? "ON" : "OFF";
            return whole ? String.valueOf((long) get()) : String.format("%.2f", get());
        }
    }

    public static final List<Item> ITEMS = new ArrayList<>();

    static {
        ITEMS.add(new Item("shot", "Shot speed", 5, 200, 2800, true,
                () -> ShotSequencer.SHOT_VELOCITY, v -> ShotSequencer.SHOT_VELOCITY = (int) v));
        ITEMS.add(new Item("shotWait", "Shot: wait for wheel between balls", 1, 0, 1, true,
                () -> ShotSequencer.WAIT_BETWEEN_BALLS ? 1 : 0, v -> ShotSequencer.WAIT_BETWEEN_BALLS = v >= 0.5));
        ITEMS.add(new Item("shotFeed", "Feeder speed when shooting", 100, 100, 2800, true,
                () -> ShotSequencer.FEED_VELOCITY, v -> ShotSequencer.FEED_VELOCITY = (int) v));
        ITEMS.add(new Item("shotWindow", "Shot ready window (+/-)", 10, 10, 300, true,
                () -> ShotSequencer.READY_TOLERANCE, v -> ShotSequencer.READY_TOLERANCE = v));
        ITEMS.add(new Item("intake", "Intake speed", 50, 100, 2500, true,
                () -> Intake.IN_VELOCITY, v -> Intake.IN_VELOCITY = (int) v));
        ITEMS.add(new Item("feedUp", "Feeder speed (cross)", 100, 100, 2800, true,
                () -> Feeder.UP_VELOCITY, v -> Feeder.UP_VELOCITY = (int) v));
        ITEMS.add(new Item("driveMax", "Drive top speed", 0.05, 0.2, 1, false,
                () -> Drivetrain.MAX_SPEED, v -> Drivetrain.MAX_SPEED = v));
        ITEMS.add(new Item("turnMax", "Turn speed", 0.05, 0.1, 1, false,
                () -> Drivetrain.TURN_SPEED, v -> Drivetrain.TURN_SPEED = v));
        ITEMS.add(new Item("ramp", "Speed-up time (s)", 0.1, 0, 1.5, false,
                () -> Drivetrain.RAMP_SECONDS, v -> Drivetrain.RAMP_SECONDS = v));
        ITEMS.add(new Item("fieldCentric", "Field-centric at start", 1, 0, 1, true,
                () -> BaseTeleOp.FIELD_CENTRIC_DEFAULT ? 1 : 0, v -> BaseTeleOp.FIELD_CENTRIC_DEFAULT = v >= 0.5));
        ITEMS.add(new Item("headingHold", "Heading hold at start", 1, 0, 1, true,
                () -> BaseTeleOp.HEADING_HOLD_DEFAULT ? 1 : 0, v -> BaseTeleOp.HEADING_HOLD_DEFAULT = v >= 0.5));
        ITEMS.add(new Item("autoDelay", "Auto: wait before moving (s)", 0.5, 0, 15, false,
                () -> BaseAuto.START_DELAY_S, v -> BaseAuto.START_DELAY_S = v));
        ITEMS.add(new Item("autoLeave", "Auto: leave distance (in)", 1, 0, 48, true,
                () -> BaseAuto.LEAVE_DISTANCE, v -> BaseAuto.LEAVE_DISTANCE = v));
        ITEMS.add(new Item("parkForward", "Auto park: forward (in)", 1, -72, 72, true,
                () -> ParkAuto.PARK_FORWARD, v -> ParkAuto.PARK_FORWARD = v));
        ITEMS.add(new Item("parkSide", "Auto park: sideways (in, + = left on red)", 1, -72, 72, true,
                () -> ParkAuto.PARK_SIDE, v -> ParkAuto.PARK_SIDE = v));
        addSpot("redShoot",   "RED A shoot spot",    AutoSpots.RED_SHOOT);
        addSpot("redEnd",     "RED A end spot",      AutoSpots.RED_END);
        addSpot("blueShoot",  "BLUE A shoot spot",   AutoSpots.BLUE_SHOOT);
        addSpot("blueEnd",    "BLUE A end spot",     AutoSpots.BLUE_END);
        addSpot("redShootB",  "RED B shoot spot",    AutoSpots.RED_SHOOT_B);
        addSpot("redEndB",    "RED B end spot",      AutoSpots.RED_END_B);
        addSpot("blueShootB", "BLUE B shoot spot",   AutoSpots.BLUE_SHOOT_B);
        addSpot("blueEndB",   "BLUE B end spot",     AutoSpots.BLUE_END_B);
        addSpot("redPark",    "RED safe park spot",  AutoSpots.RED_PARK);
        addSpot("bluePark",   "BLUE safe park spot", AutoSpots.BLUE_PARK);
    }

    private static void addSpot(String key, String label, double[] spot) {
        ITEMS.add(new Item(key + "X", label + ": ahead (in)", 1, -144, 144, true, () -> spot[0], v -> spot[0] = v));
        ITEMS.add(new Item(key + "Y", label + ": left (in)", 1, -144, 144, true, () -> spot[1], v -> spot[1] = v));
        ITEMS.add(new Item(key + "H", label + ": turn (deg)", 5, -180, 180, true, () -> spot[2], v -> spot[2] = v));
    }

    /** Copies the saved file into the code. Missing file or values = code defaults. Returns false on error. */
    public static boolean load() {
        if (!FILE.exists()) return true;
        Properties p = new Properties();
        try (InputStream in = new FileInputStream(FILE)) {
            p.load(in);
        } catch (Exception e) {
            return false;
        }
        for (Item item : ITEMS) {
            String s = p.getProperty(item.key);
            if (s == null) continue;
            try {
                item.set(Double.parseDouble(s));
            } catch (NumberFormatException ignored) {
                // a bad value keeps the current one
            }
        }
        for (String name : p.stringPropertyNames()) {
            if (name.startsWith("path.")) AutoSpots.decodePoints(name.substring(5), p.getProperty(name));
        }
        return true;
    }

    public static boolean save() {
        Properties p = new Properties();
        for (Item item : ITEMS) p.setProperty(item.key, String.valueOf(item.get()));
        for (String route : AutoSpots.routes()) p.setProperty("path." + route, AutoSpots.encodePoints(route));
        try (OutputStream out = new FileOutputStream(FILE)) {
            p.store(out, "BIOBUZZ robot settings (edit with the settingsMenu OpMode)");
            return true;
        } catch (Exception e) {
            return false;
        }
    }
}
