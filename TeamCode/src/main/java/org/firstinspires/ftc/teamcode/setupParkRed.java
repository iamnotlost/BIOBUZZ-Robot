package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Teach the safe park auto (parkRed): INIT on the CORNER start spot, drive into the LOADING ZONE, press O
@TeleOp(name="setupParkRed", group = "setup")
public class setupParkRed extends baseSetupAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected boolean parkOnly() {
        return true;
    }
}
