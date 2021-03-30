package org.firstinspires.ftc.team15021.gamepad;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The SpacedTicks strategy helps with implementation of things like scrolling where the user would
 * prefer to simply hold a button for a defined period in order to change a value or press it multiple
 * times instead of pressing the button multiple times.
 */
public class SpacedTicks implements PolledStrategy {
    protected boolean state = false; // Strategy state - true when interval has passed.
    protected boolean previous = false; // The previous button state.
    protected boolean polled = false; // Whether or not the strategy has been polled yet.

    protected double interval; // Interval determines time that must pass before the state fires.
    protected ElapsedTime time; // ElapsedTime instance to help track how long it's been running.
    protected double waitUntil; // The time that should pass before state should activate again.

    /**
     * Defaults to 0.25 second activation interval.
     */
    SpacedTicks() {
        this(0.25);
    }

    /**
     * @param interval The amount of time that should pass in between each state activation.
     */
    SpacedTicks(double interval) {
        this.interval = Math.max(0, interval);
        time = new ElapsedTime();
    }

    @Override
    public void update(boolean next) {
        // Reset the state and polled state at the start of every tick.
        state = false;
        polled = false;

        // Button is depressed, start or continue firing
        if (next) {
            // Not initial press. Check if interval has passed.
            if (previous)
                // If enough time has elapsed
                if (time.seconds() >= waitUntil) {
                    state = true;
                }
            else {
                // Initial press, fire and reset time elapsed as we start tracking.
                time.reset();
                state = true;
            }
        }

        // Move the next firing time up by the selected interval each time we activate.
        if (state)
            waitUntil = time.seconds() + interval;

        // Remember previous button state.
        previous = next;
    }

    @Override
    public boolean poll() {
        if (polled)
            return false;
        else {
            polled = true;
            return state;
        }
    }

    @Override
    public boolean peek() {
        return state;
    }
}
