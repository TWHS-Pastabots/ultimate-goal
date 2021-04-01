package org.firstinspires.ftc.teamcode.gamepad;

/**
 * The Toggle strategy changes state between true and false each time the button is pressed.
 * Can be configured to respond during either the button up or down tick.
 */
public class Toggle implements ListenedEventStrategy {
    private boolean changed = false;
    private boolean state = false; // Current strategy toggle state
    private boolean previous = false; // Last button state

    private boolean onDown; // Change state on the button up or down tick

    /**
     * @param onDown Whether or not the Toggle class should change state on the button up or down tick.
     */
    Toggle(boolean onDown) {
        this.onDown = onDown;
    }

    /**
     * Defaults to responding to state change during the button down tick.
     */
    public Toggle() {
        this(true);
    }

    @Override
    public void update(boolean next) {
        // Changed only lasts for 1 tick at a time.
        changed = false;

        // Change state every time the button is depressed/lifted depending on configuration
        if ((next != previous) && (onDown ? next : previous)) {
            state = !state;
            changed = true;
        }

        // Remember previous tick's button state
        previous = next;
    }

    @Override
    public boolean read() {
        return state;
    }

    @Override
    public boolean changed() {
        return changed;
    }
}
