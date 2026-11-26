package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive + Intake", group = "Bring-up")
public final class DriveAndIntake extends DriveOnly {
    @Override
    public void runOpMode() {
        runDrive(true);
    }
}
