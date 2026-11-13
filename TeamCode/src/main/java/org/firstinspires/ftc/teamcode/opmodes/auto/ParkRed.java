package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "parkRed", group = "BIOBUZZ", preselectTeleOp = "redTeleOp")
public class ParkRed extends ParkAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
