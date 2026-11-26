package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Wheel Check", group = "Bring-up")
public final class WheelCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        try {
            telemetry.addLine("Lift the wheels clear. Release bumper before testing.");
            telemetry.update();
            waitForStart();

            int selected = 0;
            boolean previousLeft = false;
            boolean previousRight = false;
            boolean armed = false;
            boolean previousA = false;
            int[] encoderZero = new int[4];
            for (int i = 0; i < 4; i++) encoderZero[i] = drive.encoderPosition(i);
            while (opModeIsActive()) {
                boolean left = gamepad1.dpad_left;
                boolean right = gamepad1.dpad_right;
                if (!gamepad1.right_bumper) {
                    drive.stop();
                    armed = true;
                    if (gamepad1.a && !previousA) {
                        for (int i = 0; i < 4; i++) encoderZero[i] = drive.encoderPosition(i);
                    }
                    if (left && !previousLeft) selected = (selected + 3) % 4;
                    if (right && !previousRight) selected = (selected + 1) % 4;
                } else if (armed) {
                    drive.testWheel(selected);
                }
                previousLeft = left;
                previousRight = right;
                previousA = gamepad1.a;
                for (int i = 0; i < 4; i++) {
                    telemetry.addData(drive.wheelName(i), "%d ticks since zero | profile %.1f ticks/output rev",
                            drive.encoderPosition(i) - encoderZero[i],
                            drive.configuredTicksPerOutputRevolution(i));
                }
                telemetry.addLine("Release bumper and press A to zero the display.");
                telemetry.addData("Selected wheel", drive.wheelName(selected));
                telemetry.addLine("D-pad left/right selects. Hold right bumper to test at 15%.");
                telemetry.update();
                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
