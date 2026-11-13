package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "shootBlue", group = "BIOBUZZ", preselectTeleOp = "blueTeleOp")
public class ShootBlue extends ShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
