package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

final class PedroChassisCalibration {
    static final double TEST_POWER = 0.25;
    // Record values from this year's robot. See DRIVETRAIN_BRINGUP.md.
    static final boolean VERIFIED = false;
    static final double TRACK_WIDTH_IN = Double.NaN;
    static final double WHEELBASE_IN = Double.NaN;
    static final double FORWARD_IN_PER_SUM_TICK = Double.NaN;
    static final double STRAFE_IN_PER_SUM_TICK = Double.NaN;
    static final double TURN_SCALE = Double.NaN;
    static final double FORWARD_SPEED_IN_PER_SEC = Double.NaN;
    static final double STRAFE_SPEED_IN_PER_SEC = Double.NaN;

    static boolean ready() {
        if (!VERIFIED) return false;
        for (double value : new double[]{TRACK_WIDTH_IN, WHEELBASE_IN, FORWARD_IN_PER_SUM_TICK,
                STRAFE_IN_PER_SUM_TICK, TURN_SCALE, FORWARD_SPEED_IN_PER_SEC, STRAFE_SPEED_IN_PER_SEC}) {
            if (!Double.isFinite(value) || value <= 0) return false;
        }
        return true;
    }

    static Follower create(HardwareMap map) {
        if (!ready()) throw new IllegalStateException("Pedro chassis calibration is incomplete");
        String[] names = MecanumDrive.NAMES;
        DcMotorSimple.Direction[] directions = MecanumDrive.DIRECTIONS;
        DriveEncoderConstants encoders = new DriveEncoderConstants()
                .leftFrontMotorName(names[0]).rightFrontMotorName(names[1])
                .leftRearMotorName(names[2]).rightRearMotorName(names[3])
                .leftFrontEncoderDirection(encoderSign(directions[0]))
                .rightFrontEncoderDirection(encoderSign(directions[1]))
                .leftRearEncoderDirection(encoderSign(directions[2]))
                .rightRearEncoderDirection(encoderSign(directions[3]))
                .robotWidth(TRACK_WIDTH_IN).robotLength(WHEELBASE_IN)
                .forwardTicksToInches(FORWARD_IN_PER_SUM_TICK)
                .strafeTicksToInches(STRAFE_IN_PER_SUM_TICK)
                .turnTicksToInches(TURN_SCALE);
        MecanumConstants drivetrain = new MecanumConstants()
                .leftFrontMotorName(names[0]).rightFrontMotorName(names[1])
                .leftRearMotorName(names[2]).rightRearMotorName(names[3])
                .leftFrontMotorDirection(directions[0]).rightFrontMotorDirection(directions[1])
                .leftRearMotorDirection(directions[2]).rightRearMotorDirection(directions[3])
                .xVelocity(FORWARD_SPEED_IN_PER_SEC).yVelocity(STRAFE_SPEED_IN_PER_SEC)
                .maxPower(TEST_POWER);
        return new FollowerBuilder(new FollowerConstants(), map)
                .driveEncoderLocalizer(encoders).mecanumDrivetrain(drivetrain).build();
    }

    // Pedro 2.0.6 applies the SDK motor direction again. Cancel that extra sign
    // so its wheel deltas match the SDK readings shown in Wheel Check.
    private static double encoderSign(DcMotorSimple.Direction direction) {
        return direction == DcMotorSimple.Direction.REVERSE ? Encoder.REVERSE : Encoder.FORWARD;
    }
}
