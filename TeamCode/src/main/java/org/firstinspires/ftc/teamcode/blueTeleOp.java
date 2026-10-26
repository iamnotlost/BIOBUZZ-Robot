package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="blueTeleOp", group = "testing")
public class blueTeleOp extends baseTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
