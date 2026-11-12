package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="setupAutoRed", group = "setup")
public class setupAutoRed extends baseSetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
