package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="parkRed", group = "BIOBUZZ")
public class parkRed extends baseParkAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
