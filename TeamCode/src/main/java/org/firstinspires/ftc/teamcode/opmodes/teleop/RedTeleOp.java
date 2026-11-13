package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "redTeleOp", group = "match")
public class RedTeleOp extends BaseTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
