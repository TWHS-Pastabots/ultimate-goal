package org.firstinspires.ftc.teamcode.gamepad;

/**
 * The SingleDown strategy fires on the tick that the button is pressed down.
 */
public class SingleDown implements PolledStrategy {
    private boolean previous = false; // The last state of the button.
    private boolean state = false; // The current state of the strategy.
    private boolean polled = false; // Whether or not the current variable state has been read.

    public SingleDown() {}

    @Override
    public void update(boolean next) {
        // If the last state was off and it's now on, the key was pressed.
        state = !previous && next;

        // Clear polled state.
        polled = false;

        previous = next;
    }

    /**
     * Returns gamepad button state. Polling can only return once in a row per call to update.
     *
     * @return Returns the state of the gamepad button.
     * @see #update(boolean)
     */
    public boolean poll() {
        if (polled)
            return false;
        else {
            polled = true;
            return state;
        }
    }

    /**
     * @return Returns the true state of the gamepad button state.
     */
    public boolean peek() {
        return state;
    }
}
