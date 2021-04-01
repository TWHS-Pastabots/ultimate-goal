package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The Lockup strategy is used specifically for ensuring gamepad switching does not accidentally
 * activate buttons. This is done by simply activating the 'lockup' state for a defined period
 * after the Options (PS4 controller) button is pressed.
 */
public class Lockup implements RawBooleanStrategy {
    private ElapsedTime time; // Amount of time passed since init or initial button press.
    private double lockupTime; // Amount of time that state should be active for after button press.

    private boolean previous; // The previous button state.
    private boolean state; // Whether lockup is active or not.

    public Lockup() {
        this(1.5);
    }

    public Lockup(double lockupTime) {
        this.lockupTime = lockupTime;
        time = new ElapsedTime();
    }

    @Override
    public void update(boolean next) {
        if (state) {
            // If state is active, we ignore if the button changed or not.
            if (time.seconds() >= lockupTime) {
                // Lockup time has passed, 'unlock'.
                state = false;
            }
        } else if (next && !previous) {
            // If lockup is not active but the button has been just pressed, lockup and start timing.
            time.reset();
            state = true;
        }

        // Remember current state
        previous = next;
    }

    @Override
    public boolean read() {
        return state;
    }
}
