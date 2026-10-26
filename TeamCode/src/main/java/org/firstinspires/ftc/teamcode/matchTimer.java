package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

// TELEOP clock for BIOBUZZ. Start it when the TELEOP OpMode is started (at the "Drivers, pick up
// your controllers" cue) so it lines up with the field timer.
public class matchTimer {
    public static final double TELEOP_SECONDS        = 120;
    public static final double NECTAR_FLOWER_SECONDS = 60;   // G410: NECTAR into FLOWERS only in the last 60 s
    public static final double FINAL_SECONDS         = 20;

    // G404: no powered movement after the match ends. Set false to practice without the 2:00 cutoff.
    public static boolean ENFORCE_MATCH_END = true;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean started = false;
    private boolean nectarWarned = false;
    private boolean finalWarned = false;

    public void start() {
        timer.reset();
        started = true;
        nectarWarned = false;
        finalWarned = false;
    }

    public double remaining() {
        if (!started) return TELEOP_SECONDS;
        return Math.max(0, TELEOP_SECONDS - timer.seconds());
    }

    public boolean nectarFlowerAllowed() {
        return started && remaining() <= NECTAR_FLOWER_SECONDS;
    }

    public boolean isOver() {
        return ENFORCE_MATCH_END && started && remaining() <= 0;
    }

    // Rumbles the gamepads once at 1:00 left (NECTAR can go into FLOWERS) and once at 0:20 left
    public void update(Gamepad... gamepads) {
        if (!started) return;
        if (!nectarWarned && remaining() <= NECTAR_FLOWER_SECONDS) {
            nectarWarned = true;
            for (Gamepad g : gamepads) g.rumbleBlips(2);
        }
        if (!finalWarned && remaining() <= FINAL_SECONDS) {
            finalWarned = true;
            for (Gamepad g : gamepads) g.rumbleBlips(3);
        }
    }

    public String display() {
        int s = (int) Math.ceil(remaining());
        return String.format("%d:%02d", s / 60, s % 60);
    }
}
