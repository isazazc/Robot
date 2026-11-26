package org.firstinspires.ftc.teamcode;

/** Wall-local survey. Positive X is inward and positive Y is left when facing inward. */
final class BiobuzzAutoRoute {
    static final double POSITION_TOLERANCE_IN = 25.0 / 25.4;
    static final double HEADING_TOLERANCE_RAD = Math.toRadians(3);
    private static final double PARK_MARGIN_IN = 50.0 / 25.4;
    private static final double START_TOLERANCE_IN = 0.25;

    // Enter separate measured surveys for the two alliance routes. Null locks that route.
    static final BiobuzzAutoRoute RED = null;
    static final BiobuzzAutoRoute BLUE = null;

    static final class Point {
        final double x;
        final double y;

        Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    static final class Bounds {
        final double minX;
        final double minY;
        final double maxX;
        final double maxY;

        Bounds(double minX, double minY, double maxX, double maxY) {
            this.minX = minX;
            this.minY = minY;
            this.maxX = maxX;
            this.maxY = maxY;
        }

        boolean valid() {
            return finite(minX, minY, maxX, maxY) && minX < maxX && minY < maxY;
        }
    }

    private final double length;
    private final double width;
    final Point start;
    final Point leave;
    final Point park;
    private final Bounds loadingZone;
    private final Bounds corridor;
    final double leaveTimeout;
    final double parkTimeout;

    BiobuzzAutoRoute(double length, double width, Point start, Point leave, Point park,
                    Bounds loadingZone, Bounds corridor, double leaveTimeout, double parkTimeout) {
        this.length = length;
        this.width = width;
        this.start = start;
        this.leave = leave;
        this.park = park;
        this.loadingZone = loadingZone;
        this.corridor = corridor;
        this.leaveTimeout = leaveTimeout;
        this.parkTimeout = parkTimeout;
    }

    String validationError() {
        if (!finite(length, width, start.x, start.y, leave.x, leave.y, park.x, park.y,
                leaveTimeout, parkTimeout) || length <= 0 || width <= 0
                || !loadingZone.valid() || !corridor.valid()) {
            return "Survey contains invalid dimensions";
        }
        if (leaveTimeout <= 0 || parkTimeout <= 0
                || leaveTimeout + parkTimeout + BiobuzzAutoSequence.SETTLE_SECONDS > BiobuzzAutoSequence.TARGET_SECONDS) {
            return "Segment budgets must leave time to settle by 25 s";
        }
        if (!fits(start.x, start.y, 0) || !fits(leave.x, leave.y, 0) || !fits(park.x, park.y, 0)) {
            return "Planned footprint leaves the surveyed clear corridor";
        }
        if (Math.abs(start.x - length / 2) > START_TOLERANCE_IN
                || Math.abs(leave.y - start.y) > 1e-6 || leave.x - length / 2 < PARK_MARGIN_IN) {
            return "Start at the wall and leave straight inward";
        }
        double startOverlapX = overlap(start.x, length, loadingZone.minX, loadingZone.maxX);
        double startOverlapY = overlap(start.y, width, loadingZone.minY, loadingZone.maxY);
        if (startOverlapX >= 0 && startOverlapY >= 0) {
            return "Starting footprint touches or overlaps the loading zone";
        }
        if (park.x - length / 2 < PARK_MARGIN_IN
                || overlap(park.x, length, loadingZone.minX, loadingZone.maxX) < PARK_MARGIN_IN
                || overlap(park.y, width, loadingZone.minY, loadingZone.maxY) < PARK_MARGIN_IN
                || Math.hypot(park.x - leave.x, park.y - leave.y) <= POSITION_TOLERANCE_IN) {
            return "Park needs 50 mm wall clearance and zone overlap with a distinct endpoint";
        }
        return null;
    }

    boolean fits(double x, double y, double heading) {
        double cosine = Math.abs(Math.cos(heading));
        double sine = Math.abs(Math.sin(heading));
        double halfX = (length * cosine + width * sine) / 2;
        double halfY = (length * sine + width * cosine) / 2;
        return x - halfX >= corridor.minX - 1e-6 && x + halfX <= corridor.maxX + 1e-6
                && y - halfY >= corridor.minY - 1e-6 && y + halfY <= corridor.maxY + 1e-6;
    }

    static boolean within(double x, double y, double heading, Point target) {
        double wrappedHeading = Math.atan2(Math.sin(heading), Math.cos(heading));
        return Math.hypot(x - target.x, y - target.y) <= POSITION_TOLERANCE_IN
                && Math.abs(wrappedHeading) <= HEADING_TOLERANCE_RAD;
    }

    static boolean finite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) return false;
        }
        return true;
    }

    private static double overlap(double centre, double size, double minimum, double maximum) {
        return Math.min(centre + size / 2, maximum) - Math.max(centre - size / 2, minimum);
    }
}
