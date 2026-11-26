package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Only", group = "Bring-up")
public class DriveOnly extends LinearOpMode {
    private static final double INTAKE_POWER = 0.50;

    @Override
    public void runOpMode() {
        runDrive(false);
    }

    protected final void runDrive(boolean useIntake) {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        DcMotor intake = null;
        try {
            if (useIntake) {
                intake = hardwareMap.get(DcMotor.class, "intakeMotor");
                intake.setPower(0);
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Cross (A) toggles Pollen intake. STOP turns it off.");
            }
            telemetry.addLine("Centre sticks and release right bumper before enabling drive.");
            telemetry.addLine("Hold right bumper to drive. Left stick moves, right stick turns.");
            telemetry.update();
            waitForStart();

            boolean enabled = false;
            boolean intakeOn = false;
            boolean previousA = true; // Require a fresh press after START.
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
                if (intake != null) {
                    boolean a = gamepad1.a;
                    if (a && !previousA) {
                        intakeOn = !intakeOn;
                        intake.setPower(intakeOn ? INTAKE_POWER : 0);
                    }
                    previousA = a;
                    telemetry.addData("Intake (Cross/A)", intakeOn ? "ON" : "OFF");
                }
                telemetry.addData("Drive", enabled && gamepad1.right_bumper
                        ? "Active" : "Centre sticks, release, then hold right bumper");
                telemetry.update();
                idle();
            }
        } finally {
            try {
                if (intake != null) intake.setPower(0);
            } finally {
                drive.stop();
            }
        }
    }
}
