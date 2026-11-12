package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="setupAutoBlue", group = "setup")
public class setupAutoBlue extends baseSetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
