package org.firstinspires.ftc.teamcode.opmodes.setup;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "setupAutoRed", group = "setup")
public class SetupAutoRed extends SetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
