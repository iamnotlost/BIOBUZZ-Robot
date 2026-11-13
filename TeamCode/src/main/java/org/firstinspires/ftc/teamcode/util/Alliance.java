package org.firstinspires.ftc.teamcode.util;

/** Which side we play on. {@link #sign} mirrors left/right distances: red +, blue -. */
public enum Alliance {
    RED(1),
    BLUE(-1);

    public final int sign;

    Alliance(int sign) {
        this.sign = sign;
    }

    /** "Red" / "Blue", for OpMode names in messages. */
    public String title() {
        return this == RED ? "Red" : "Blue";
    }
}
