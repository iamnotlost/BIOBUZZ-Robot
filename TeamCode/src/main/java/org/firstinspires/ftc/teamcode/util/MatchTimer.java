package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TELEOP clock. Start it when the TELEOP OpMode starts so it lines up with the field timer.
 * Rumbles the gamepads at 1:00 left (NECTAR may go into FLOWERS, G410) and at 0:15 left (go park),
 * and reports when the match is over (G404: no powered movement after the end).
 */
public class MatchTimer {
    public static final double TELEOP_SECONDS        = 120;
    public static final double NECTAR_FLOWER_SECONDS = 60;
    public static final double PARK_SECONDS          = 15;

    /** Set false to practice without the 2:00 cutoff. */
    public static boolean ENFORCE_MATCH_END = true;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean started, nectarWarned, parkWarned;

    public void start() {
        timer.reset();
        started = true;
        nectarWarned = false;
        parkWarned = false;
    }

    public double remaining() {
        return started ? Math.max(0, TELEOP_SECONDS - timer.seconds()) : TELEOP_SECONDS;
    }

    public boolean nectarFlowerAllowed() {
        return started && remaining() <= NECTAR_FLOWER_SECONDS;
    }

    public boolean parkTime() {
        return started && remaining() <= PARK_SECONDS;
    }

    public boolean isOver() {
        return ENFORCE_MATCH_END && started && remaining() <= 0;
    }

    /** Call every loop: 2 short buzzes at 1:00, one long buzz at 0:15. */
    public void update(Gamepad... gamepads) {
        if (!started) return;
        if (!nectarWarned && remaining() <= NECTAR_FLOWER_SECONDS) {
            nectarWarned = true;
            for (Gamepad g : gamepads) g.rumbleBlips(2);
        }
        if (!parkWarned && remaining() <= PARK_SECONDS) {
            parkWarned = true;
            for (Gamepad g : gamepads) g.rumble(1000);
        }
    }

    public String display() {
        int s = (int) Math.ceil(remaining());
        return String.format("%d:%02d", s / 60, s % 60);
    }
}
