package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "parkBlue", group = "BIOBUZZ", preselectTeleOp = "blueTeleOp")
public class ParkBlue extends ParkAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
