package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "Auto")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        try {
            telemetry.addLine("Timed straight drive: 30% power for 1.5 seconds.");
            telemetry.update();
            waitForStart();

            ElapsedTime elapsed = new ElapsedTime();
            while (opModeIsActive() && elapsed.seconds() < 1.5) {
                drive.drive(1, 0, 0, 0.30);
                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
