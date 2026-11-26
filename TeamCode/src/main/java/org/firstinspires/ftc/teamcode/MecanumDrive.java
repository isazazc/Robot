package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

final class MecanumDrive {
    static final double STICK_DEADBAND = 0.06;
    static final String[] NAMES = {"frontLeft", "frontRight", "backLeft", "backRight"};
    // Inherited from last year's code. Verify each direction on the current chassis.
    static final DcMotorSimple.Direction[] DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD
    };

    private final DcMotor[] wheels = new DcMotor[NAMES.length];

    MecanumDrive(HardwareMap hardwareMap) {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i] = hardwareMap.get(DcMotor.class, NAMES[i]);
            wheels[i].setPower(0);
            wheels[i].setDirection(DIRECTIONS[i]);
            wheels[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheels[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    static double stick(double value) {
        double magnitude = Math.abs(value);
        if (!Double.isFinite(value) || magnitude <= STICK_DEADBAND) {
            return 0;
        }
        return Math.copySign((Math.min(1, magnitude) - STICK_DEADBAND)
                / (1 - STICK_DEADBAND), value);
    }

    static double[] powers(double forward, double right, double clockwise, double limit) {
        if (!Double.isFinite(forward) || !Double.isFinite(right)
                || !Double.isFinite(clockwise) || !Double.isFinite(limit)) {
            return new double[4];
        }
        double divisor = Math.max(1, Math.abs(forward) + Math.abs(right) + Math.abs(clockwise));
        double scale = Math.max(0, Math.min(1, limit)) / divisor;
        return new double[]{
                (forward + right + clockwise) * scale,
                (forward - right - clockwise) * scale,
                (forward - right + clockwise) * scale,
                (forward + right - clockwise) * scale
        };
    }

    void drive(double forward, double right, double clockwise, double limit) {
        double[] powers = powers(forward, right, clockwise, limit);
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(powers[i]);
        }
    }

    int encoderPosition(int index) {
        return wheels[index].getCurrentPosition();
    }

    double configuredTicksPerOutputRevolution(int index) {
        return wheels[index].getMotorType().getTicksPerRev();
    }

    String wheelName(int index) {
        return NAMES[index];
    }

    void testWheel(int index) {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower(i == index ? 0.15 : 0);
        }
    }

    void brake() {
        stop();
        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    void stop() {
        for (DcMotor wheel : wheels) {
            wheel.setPower(0);
        }
    }
}
