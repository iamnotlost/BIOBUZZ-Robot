package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="parkBlue", group = "BIOBUZZ", preselectTeleOp = "blueTeleOp")
public class parkBlue extends baseParkAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
