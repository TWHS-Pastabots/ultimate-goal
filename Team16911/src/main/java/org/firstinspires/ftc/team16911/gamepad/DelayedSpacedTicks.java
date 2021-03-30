package org.firstinspires.ftc.team16911.gamepad;

/**
 * A extension on the SpacedTicks class, this adds a configurable delay that must pass before the
 * interval will begin activating.
 */
public class DelayedSpacedTicks extends SpacedTicks implements PolledStrategy {
    private boolean initialActivate; // Whether or not it should activate on the initial button press.
    private double delay; // The amount of time that must pass before activations occur on an interval.

    private boolean delayPassed = false; // Has the delay stage passed yet?
    private double delayTime; // The total elapsed time that must pass before the delay stage ends.

    /**
     * Defaults to a 0.3 second delay and a 0.25 second activation interval.
     */
    DelayedSpacedTicks() {
        this(0.3, 0.25);
    }

    /**
     * @param delay    The amount of time that must pass before activations occur on an interval.
     * @param interval The amount of time that should pass in between each state activation.
     */
    DelayedSpacedTicks(double delay, double interval) {
        super(interval);
        this.delay = Math.max(0, delay);
    }

    /**
     * @param delay           The amount of time that must pass before activations occur on an interval.
     * @param interval        The amount of time that should pass in between each state activation.
     * @param initialActivate Whether or not it should activate on the initial button press.
     */
    DelayedSpacedTicks(double delay, double interval, boolean initialActivate) {
        this(delay, interval);
        this.initialActivate = initialActivate;
    }

    @Override
    public void update(boolean next) {
        // Use SpacedTicks implementation since it would be identical.
        if (initialActivate && delay <= 0) {
            super.update(next);
            return;
        }

        // Reset the state and polled state at the start of every tick.
        state = false;
        polled = false;

        // Button is depressed, start or continue firing
        if (next) {

            if (previous) {
                // Not initial press. Check if interval has passed.
                double cur = time.seconds();
                if (delayPassed && cur >= waitUntil) {
                    // Check if interval has passed if the delay already has.
                    state = true;
                } else if (!delayPassed && cur >= delayTime) {
                    // Check if the delay has passed yet
                    state = true;
                    delayPassed = true;
                }
            } else {
                // Initial press, fire and reset time elapsed as we start tracking.
                time.reset();
                state = initialActivate;

                // Setup delay
                delayPassed = false;
                delayTime = time.seconds() + delay;
            }
        }

        // Move the next firing time up by the selected interval each time we activate.
        if (state)
            waitUntil = time.seconds() + interval;

        // Remember previous button state.
        previous = next;
    }
}

