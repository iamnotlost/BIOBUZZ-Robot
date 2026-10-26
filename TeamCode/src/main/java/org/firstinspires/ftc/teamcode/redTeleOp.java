package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="redTeleOp", group = "testing")
public class redTeleOp extends baseTeleOp {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
