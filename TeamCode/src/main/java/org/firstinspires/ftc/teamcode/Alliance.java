package org.firstinspires.ftc.teamcode;

public enum Alliance {
    RED(1),
    BLUE(-1);

    // Mirrors headings/positions across the field: red uses +, blue uses -
    public final int sign;

    Alliance(int sign) {
        this.sign = sign;
    }
}
