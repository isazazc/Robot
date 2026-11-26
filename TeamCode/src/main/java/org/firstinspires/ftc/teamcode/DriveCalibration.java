package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Drive Calibration", group = "Bring-up")
public final class DriveCalibration extends LinearOpMode {
    private enum State { WAIT_FOR_ZERO, READY, DRIVING, SETTLING, RESULT }

    private static final double POWER_LIMIT = 0.20;
    private static final double SETTLE_SECONDS = 0.3;

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        try {
            telemetry.addLine("Mark the start pose. Press START, then A with the bumper released.");
            telemetry.update();
            waitForStart();

            State state = State.WAIT_FOR_ZERO;
            boolean previousA = true;
            int[] baseline = new int[4];
            int[] deltas = new int[4];
            ElapsedTime stableTime = new ElapsedTime();

            while (opModeIsActive()) {
                double forward = MecanumDrive.stick(-gamepad1.left_stick_y);
                double right = MecanumDrive.stick(gamepad1.left_stick_x);
                double turn = MecanumDrive.stick(gamepad1.right_stick_x);
                boolean bumper = gamepad1.right_bumper;
                boolean zeroPressed = gamepad1.a && !previousA;
                previousA = gamepad1.a;

                if (state == State.DRIVING && !bumper) {
                    drive.stop();
                    state = State.SETTLING;
                    stableTime.reset();
                }
                if ((state == State.WAIT_FOR_ZERO || state == State.READY || state == State.RESULT)
                        && zeroPressed && !bumper && forward == 0 && right == 0 && turn == 0) {
                    for (int i = 0; i < baseline.length; i++) {
                        baseline[i] = drive.encoderPosition(i);
                        deltas[i] = 0;
                    }
                    state = State.READY;
                }

                if (state == State.READY && bumper) state = State.DRIVING;
                if (state == State.DRIVING) drive.drive(forward, right, turn, POWER_LIMIT);

                if (state == State.DRIVING || state == State.SETTLING) {
                    for (int i = 0; i < deltas.length; i++) {
                        int next = drive.encoderPosition(i) - baseline[i];
                        if (next != deltas[i]) stableTime.reset();
                        deltas[i] = next;
                    }
                    if (state == State.SETTLING && stableTime.seconds() >= SETTLE_SECONDS) {
                        state = State.RESULT;
                    }
                }

                telemetry.addData("Calibration", state);
                if (state != State.WAIT_FOR_ZERO) {
                    for (int i = 0; i < deltas.length; i++) {
                        telemetry.addData(drive.wheelName(i), "%+d ticks", deltas[i]);
                    }
                }
                telemetry.addLine("With centred sticks and bumper released, press A to zero and arm.");
                telemetry.addLine("Hold right bumper to drive (20% max). Release to capture.");
                telemetry.addLine("At RESULT, record the ticks and measure the actual distance or turn.");
                telemetry.update();
                idle();
            }
        } finally {
            drive.brake();
        }
    }
}
