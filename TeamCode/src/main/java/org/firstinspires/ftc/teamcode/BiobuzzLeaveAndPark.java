package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "BIOBUZZ: Leave and Park", group = "BIOBUZZ")
public final class BiobuzzLeaveAndPark extends LinearOpMode {
    private static final String LOG_TAG = "BioBuzzAuto";

    @Override
    public void runOpMode() {
        boolean blue = false;
        boolean confirmed = false;
        boolean previousA = true;
        BiobuzzAutoRoute route = BiobuzzAutoRoute.RED;
        String lock = lockedReason(route);
        while (opModeInInit()) {
            boolean selectedBlue = blue;
            if (gamepad1.dpad_left) selectedBlue = false;
            else if (gamepad1.dpad_right) selectedBlue = true;
            if (selectedBlue != blue) confirmed = false;
            blue = selectedBlue;
            route = blue ? BiobuzzAutoRoute.BLUE : BiobuzzAutoRoute.RED;
            lock = lockedReason(route);
            if (gamepad1.a && !previousA && lock == null) confirmed = true;
            previousA = gamepad1.a;

            telemetry.addData("Alliance route", blue ? "Blue" : "Red");
            telemetry.addData("Calibration", PedroChassisCalibration.ready() ? "Ready" : "Missing");
            telemetry.addData("Status", lock != null ? lock : confirmed ? "Armed" : "Verify setup and press A");
            telemetry.addLine("Left = Red, Right = Blue. A confirms start and partner parking slot.");
            if (route != null) {
                telemetry.addData("Start (in)", "%.1f, %.1f", route.start.x, route.start.y);
                telemetry.addData("Park (in)", "%.1f, %.1f", route.park.x, route.park.y);
            }
            telemetry.update();
            idle();
        }

        ElapsedTime clock = new ElapsedTime();
        if (!opModeIsActive()) return;
        if (!confirmed || lock != null) {
            telemetry.addData("AUTO locked", lock == null ? "INIT confirmation missing" : lock);
            telemetry.update();
            return;
        }
        runRoute(route, clock);
    }

    private static String lockedReason(BiobuzzAutoRoute route) {
        if (!PedroChassisCalibration.ready()) return "Chassis calibration is incomplete";
        if (route == null) return "Selected alliance route has not been surveyed";
        return route.validationError();
    }

    private void runRoute(BiobuzzAutoRoute route, ElapsedTime clock) {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Follower follower = null;
        try {
            follower = PedroChassisCalibration.create(hardwareMap);
            follower.setMaxPower(PedroChassisCalibration.TEST_POWER);
            follower.setStartingPose(pose(route.start));
            follower.setPose(pose(route.start));

            Path departure = line(route.start, route.leave);
            Path parking = line(route.leave, route.park);
            BiobuzzAutoSequence sequence = new BiobuzzAutoSequence(route);
            if (!opModeIsActive() || clock.seconds() >= route.leaveTimeout) {
                telemetry.addLine("AUTO aborted before departure");
                telemetry.update();
                return;
            }
            follower.followPath(departure, false);
            logTransition(sequence, follower.getPose(), clock.seconds());
            while (true) {
                BiobuzzAutoSequence.State before = sequence.state();
                Pose current = follower.getPose();
                // Recheck time and pose before scheduling a path or updating motor outputs.
                BiobuzzAutoSequence.State after = sequence.tick(clock.seconds(), opModeIsActive(),
                        current.getX(), current.getY(), current.getHeading(), follower.isBusy());
                if (after == BiobuzzAutoSequence.State.DONE || after == BiobuzzAutoSequence.State.ABORTED) {
                    drive.brake();
                    logTransition(sequence, current, clock.seconds());
                    report(sequence, current, clock.seconds());
                    return;
                }
                if (before != after && after == BiobuzzAutoSequence.State.PARK) {
                    follower.followPath(parking, false);
                }
                if (after == BiobuzzAutoSequence.State.SETTLE) {
                    if (before != after) {
                        follower.breakFollowing();
                        drive.brake();
                    }
                    follower.updatePose();
                } else {
                    follower.update();
                }
                if (before != after) logTransition(sequence, follower.getPose(), clock.seconds());
                report(sequence, follower.getPose(), clock.seconds());
                idle();
            }
        } finally {
            try {
                if (follower != null) follower.breakFollowing();
            } finally {
                drive.brake();
            }
        }
    }

    private static void logTransition(BiobuzzAutoSequence sequence, Pose current, double seconds) {
        BiobuzzAutoRoute.Point target = sequence.target();
        RobotLog.ii(LOG_TAG,
                "t=%.2f state=%s reason=%s pose=(%.2f, %.2f, %.3f) target=(%.2f, %.2f, 0)",
                seconds, sequence.state(), sequence.reason(), current.getX(), current.getY(),
                current.getHeading(), target.x, target.y);
    }

    private void report(BiobuzzAutoSequence sequence, Pose current, double seconds) {
        BiobuzzAutoRoute.Point target = sequence.target();
        telemetry.addData("AUTO state", sequence.state());
        telemetry.addData("Reason", sequence.reason());
        telemetry.addData("Elapsed (s)", "%.2f", seconds);
        telemetry.addData("Estimate (in, deg)", "%.2f, %.2f, %.1f",
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()));
        telemetry.addData("Target (in, deg)", "%.2f, %.2f, 0", target.x, target.y);
        telemetry.update();
    }

    private static Pose pose(BiobuzzAutoRoute.Point point) {
        return new Pose(point.x, point.y, 0);
    }

    private static Path line(BiobuzzAutoRoute.Point from, BiobuzzAutoRoute.Point to) {
        Path path = new Path(new BezierLine(pose(from), pose(to)));
        path.setConstantHeadingInterpolation(0);
        return path;
    }
}
