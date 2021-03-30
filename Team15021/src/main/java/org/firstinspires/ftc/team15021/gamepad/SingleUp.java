package org.firstinspires.ftc.team15021.gamepad;

/**
 * The SingleUp Strategy fires on the tick that the button is released.
 */
public class SingleUp implements PolledStrategy {
    private boolean previous = false; // The last state of the button.
    private boolean state = false; // The current state of the button.
    private boolean polled = false; // Whether or not the current variable state has been read.

    @Override
    public void update(boolean next) {
        // If the last state was on and it's now off, the key was released.
        if (previous && !next)
            state = true;
        else {
            // Clear states.
            state = false;
            polled = false;
        }

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
