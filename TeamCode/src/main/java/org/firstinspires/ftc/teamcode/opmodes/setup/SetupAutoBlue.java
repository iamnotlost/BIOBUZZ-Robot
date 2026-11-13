package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "setupAutoBlue", group = "setup")
public class SetupAutoBlue extends SetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
