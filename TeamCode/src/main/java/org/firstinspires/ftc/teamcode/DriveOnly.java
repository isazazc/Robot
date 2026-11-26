package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive Only", group = "Bring-up")
public final class DriveOnly extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        try {
            telemetry.addLine("Centre sticks and release right bumper before enabling drive.");
            telemetry.addLine("Hold right bumper to drive. Left stick moves, right stick turns.");
            telemetry.update();
            waitForStart();

            boolean enabled = false;
            while (opModeIsActive()) {
                double forward = MecanumDrive.stick(-gamepad1.left_stick_y);
                double right = MecanumDrive.stick(gamepad1.left_stick_x);
                double turn = MecanumDrive.stick(gamepad1.right_stick_x);
                if (!gamepad1.right_bumper) {
                    drive.stop();
                    enabled = forward == 0 && right == 0 && turn == 0;
                } else if (enabled) {
                    double limit = gamepad1.left_trigger > 0.1 ? 0.20 : 0.40;
                    drive.drive(forward, right, turn, limit);
                }
                telemetry.addData("Drive", enabled && gamepad1.right_bumper
                        ? "Active" : "Centre sticks, release, then hold right bumper");
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
