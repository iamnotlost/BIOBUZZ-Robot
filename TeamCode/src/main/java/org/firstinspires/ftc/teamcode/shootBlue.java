package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="shootBlue", group = "BIOBUZZ", preselectTeleOp = "blueTeleOp")
public class shootBlue extends baseShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
