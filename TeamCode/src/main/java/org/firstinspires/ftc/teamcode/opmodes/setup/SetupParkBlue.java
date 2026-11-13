package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "setupParkBlue", group = "setup")
public class SetupParkBlue extends SetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected boolean parkOnly() {
        return true;
    }
}
