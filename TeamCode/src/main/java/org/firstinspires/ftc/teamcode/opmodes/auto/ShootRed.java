package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "shootRed", group = "BIOBUZZ", preselectTeleOp = "redTeleOp")
public class ShootRed extends ShootAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
