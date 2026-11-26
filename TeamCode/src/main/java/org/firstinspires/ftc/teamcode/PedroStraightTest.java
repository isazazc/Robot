package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pedro: Straight Test", group = "Bring-up")
public final class PedroStraightTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        if (!PedroChassisCalibration.ready()) {
            telemetry.addLine("Locked: measure and enter this chassis's calibration first.");
            telemetry.update();
            waitForStart();
            return;
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        try {
            Follower follower = PedroChassisCalibration.create(hardwareMap);
            Pose start = new Pose(0, 0, 0);
            Pose target = new Pose(12, 0, 0);
            Path path = new Path(new BezierLine(start, target));
            path.setConstantHeadingInterpolation(0);
            follower.setMaxPower(PedroChassisCalibration.TEST_POWER);

            telemetry.addLine("12 inches forward. Release and then hold right bumper to run once.");
            telemetry.update();
            waitForStart();
            boolean released = false;
            while (opModeIsActive()) {
                if (!gamepad1.right_bumper) released = true;
                else if (released) break;
                idle();
            }
            if (!opModeIsActive()) return;

            follower.setStartingPose(start);
            follower.followPath(path, false);
            ElapsedTime elapsed = new ElapsedTime();
            while (opModeIsActive() && gamepad1.right_bumper
                    && elapsed.seconds() < 5 && follower.isBusy()) {
                follower.update();
                telemetry.addData("Estimated pose (in, rad)", follower.getPose());
                telemetry.update();
                idle();
            }
            follower.breakFollowing();
            drive.brake();
            telemetry.addData("Final encoder estimate", follower.getPose());
            telemetry.addLine("Measure actual floor displacement and heading.");
            telemetry.update();
            while (opModeIsActive()) idle();
        } finally {
            drive.brake();
        }
    }
}
