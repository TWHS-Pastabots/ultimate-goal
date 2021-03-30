package org.firstinspires.ftc.team16910.gamepad;

/**
 * The Toggle strategy changes state between true and false each time the button is pressed.
 * Can be configured to respond during either the button up or down tick.
 */
public class Toggle implements RawBooleanStrategy {
    private boolean state = false; // Current strategy toggle state
    private boolean previous = false; // Last button state
    private boolean onDown = true; // Change state on the button up or down tick


    /**
     * @param onDown Whether or not the Toggle class should change state on the button up or down tick.
     */
    Toggle(boolean onDown) {
        this.onDown = onDown;
    }

    /**
     * Defaults to responding to state change during the button down tick.
     */
    Toggle() {
    }

    @Override
    public void update(boolean next) {
        // Change state every time the button is depressed/lifted depending on configuration
        if (next != previous && onDown ? previous : next)
            state = !state;

        // Remember previous tick's button state
        previous = next;
    }

    @Override
    public boolean read() {
        return state;
    }
}
