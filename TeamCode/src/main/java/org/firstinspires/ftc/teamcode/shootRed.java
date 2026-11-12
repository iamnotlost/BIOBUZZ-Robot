package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="shootRed", group = "BIOBUZZ", preselectTeleOp = "redTeleOp")
public class shootRed extends baseShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
