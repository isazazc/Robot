package org.firstinspires.ftc.teamcode;

/** Decision logic independent of FTC hardware, with latched terminal states. */
final class BiobuzzAutoSequence {
    enum State { LEAVE, PARK, SETTLE, DONE, ABORTED }

    static final double TARGET_SECONDS = 25;
    static final double HARD_STOP_SECONDS = 28;
    static final double SETTLE_SECONDS = 0.3;

    private final BiobuzzAutoRoute route;
    private State state = State.LEAVE;
    private BiobuzzAutoRoute.Point target;
    private String reason = "Following straight departure";
    private double stateStarted;

    BiobuzzAutoSequence(BiobuzzAutoRoute route) {
        this.route = route;
        target = route.leave;
    }

    State state() {
        return state;
    }

    String reason() {
        return reason;
    }

    BiobuzzAutoRoute.Point target() {
        return target;
    }

    State tick(double seconds, boolean active, double x, double y, double heading, boolean busy) {
        if (state == State.DONE || state == State.ABORTED) return state;
        if (!active) return abort("Driver Station stopped AUTO");
        if (!BiobuzzAutoRoute.finite(seconds, x, y, heading)) return abort("Invalid pose or timer");
        if (seconds >= HARD_STOP_SECONDS) return abort("28 s hard stop");
        if (!route.fits(x, y, heading)) return abort("Estimated footprint left the surveyed corridor");

        double elapsed = seconds - stateStarted;
        if (state == State.SETTLE) {
            if (!BiobuzzAutoRoute.within(x, y, heading, route.park)) {
                return abort("Parking pose drifted after braking");
            }
            if (elapsed >= SETTLE_SECONDS) {
                state = State.DONE;
                reason = "Parking estimate settled. Confirm actual overlap on the field.";
            }
        } else {
            double timeout = state == State.LEAVE ? route.leaveTimeout : route.parkTimeout;
            if (elapsed >= timeout) return abort(state + " timed out");
            if (!busy) {
                if (!BiobuzzAutoRoute.within(x, y, heading, target())) {
                    return abort(state + " ended outside target tolerance");
                }
                state = state == State.LEAVE ? State.PARK : State.SETTLE;
                target = route.park;
                stateStarted = seconds;
                reason = state == State.PARK ? "Following park path" : "Braking and checking park pose";
            }
        }
        return state;
    }

    private State abort(String cause) {
        reason = cause;
        state = State.ABORTED;
        return state;
    }
}
