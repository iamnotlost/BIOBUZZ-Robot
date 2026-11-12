package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Teach the safe park auto (parkBlue): INIT on the CORNER start spot, drive into the LOADING ZONE, press O
@TeleOp(name="setupParkBlue", group = "setup")
public class setupParkBlue extends baseSetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected boolean parkOnly() {
        return true;
    }
}
