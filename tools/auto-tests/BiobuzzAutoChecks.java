package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BiobuzzAutoSequence.State.*;

public final class BiobuzzAutoChecks {
    private static final BiobuzzAutoRoute ROUTE = route(10, 16, 0, 14, 24, 5, 6);

    public static void main(String[] args) {
        check(ROUTE.validationError() == null, "Valid surveyed route");
        check(route(12, 16, 0, 14, 24, 5, 6).validationError() != null, "Start off wall");
        check(route(10, 16, 0, 11, 24, 5, 6).validationError() != null, "Park too close to wall");
        check(route(10, 16, 13, 14, 24, 5, 6).validationError() != null, "Start overlaps zone");
        check(route(10, 16, 0, 14, 3, 5, 6).validationError() != null, "No park overlap");
        check(route(10, Double.NaN, 0, 14, 24, 5, 6).validationError() != null, "Non-finite pose");
        check(route(10, 16, 0, 14, 24, 14, 14).validationError() != null, "No time to settle");
        check(route(10, 16, 0, 14, 24, 5, 6, 25).validationError() != null, "Swept corridor boundary");
        check(!BiobuzzAutoRoute.within(13.01, 24, 0, ROUTE.park),
                "One inch is wider than the 25 mm tolerance");
        check(BiobuzzAutoRoute.within(14, 24, Math.PI * 2 - .01, ROUTE.park),
                "Wrapped heading is accepted");

        BiobuzzAutoSequence run = new BiobuzzAutoSequence(ROUTE);
        expect(run, 0, true, 10, 0, 0, true, LEAVE);
        expect(run, 2, true, 16, 0, 0, false, PARK);
        expect(run, 4, true, 14, 24, 0, false, SETTLE);
        expect(run, 4.2, true, 14, 24, 0, false, SETTLE);
        expect(run, 4.31, true, 14, 24, 0, false, DONE);
        expect(run, 29, true, 14, 24, 0, false, DONE);

        expect(new BiobuzzAutoSequence(ROUTE), 2, true, 11, 0, 0, false, ABORTED);
        expect(new BiobuzzAutoSequence(ROUTE), 5, true, 16, 0, 0, true, ABORTED);
        expect(new BiobuzzAutoSequence(ROUTE), 28, true, 16, 0, 0, false, ABORTED);
        expect(new BiobuzzAutoSequence(ROUTE), 2, true, Double.NaN, 0, 0, true, ABORTED);
        expect(new BiobuzzAutoSequence(ROUTE), 2, false, 16, 0, 0, true, ABORTED);
        expect(new BiobuzzAutoSequence(ROUTE), 2, true, 16, 0, .1, false, ABORTED);

        run = new BiobuzzAutoSequence(ROUTE);
        expect(run, 2, true, 16, 0, 0, false, PARK);
        expect(run, 4, true, 14, 24, 0, false, SETTLE);
        expect(run, 4.15, true, 13, 24, 0, false, ABORTED);
        expect(run, 4.2, true, 14, 24, 0, false, ABORTED);

        run = new BiobuzzAutoSequence(ROUTE);
        expect(run, 2, true, 16, 0, 0, false, PARK);
        expect(run, 8, true, 14, 24, 0, true, ABORTED);

        run = new BiobuzzAutoSequence(ROUTE);
        expect(run, 2, true, 16, 0, 0, false, PARK);
        expect(run, 2.1, true, 23, 0, 0, true, ABORTED);
        System.out.println("BIOBUZZ AUTO route and sequence checks passed");
    }

    private static BiobuzzAutoRoute route(double start, double leave, double south,
                                         double parkX, double parkY, double leaveTime,
                                         double parkTime) {
        return route(start, leave, south, parkX, parkY, leaveTime, parkTime, 32);
    }

    private static BiobuzzAutoRoute route(double start, double leave, double startY,
                                         double parkX, double parkY, double leaveTime,
                                         double parkTime, double corridorMaxY) {
        return new BiobuzzAutoRoute(20, 16, new BiobuzzAutoRoute.Point(start, startY),
                new BiobuzzAutoRoute.Point(leave, startY), new BiobuzzAutoRoute.Point(parkX, parkY),
                new BiobuzzAutoRoute.Bounds(0, 12, 16, 32),
                new BiobuzzAutoRoute.Bounds(0, -12, 32, corridorMaxY), leaveTime, parkTime);
    }

    private static void expect(BiobuzzAutoSequence run, double seconds, boolean active,
                               double x, double y, double heading, boolean busy,
                               BiobuzzAutoSequence.State state) {
        check(run.tick(seconds, active, x, y, heading, busy) == state,
                "Expected " + state + " at " + seconds + ", got " + run.state() + " " + run.reason());
    }

    private static void check(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
    }
}
